// #![deny(warnings)]
#![no_main]
#![no_std]


use panic_rtt_core::{self, rprint, rprintln, rtt_init_print};

// use core::cell::RefCell;
// use cortex_m::interrupt::{self, Mutex};

use stm32f4xx_hal as p_hal;

use cortex_m_rt::{entry, exception, ExceptionFrame};

use p_hal::hal::digital::v2::ToggleableOutputPin;

use freertos_sys::cmsis_rtos2;

type MsgBufType = *const cty::c_void;

#[allow(non_upper_case_globals)]
#[no_mangle]
pub static SystemCoreClock: u32 = 16_000_000; //or use stm32f4xx_hal rcc::HSI
//Can use 25_000_000 on an stm32f401 board with 25 MHz xtal
// 48_000_000 for stm32h743 HSI (48 MHz)



use p_hal::gpio::GpioExt;
use p_hal::rcc::RccExt;

use core::sync::atomic::{AtomicUsize, Ordering, AtomicPtr};

use p_hal::{prelude::*, stm32};
use core::ptr::{null, null_mut};
use cmsis_rtos2::{
  osMessageQueueId_t,
  osPriority_t,
  osThreadAttr_t,
  osPriority_t_osPriorityLow,
  osPriority_t_osPriorityNormal,
  osPriority_t_osPriorityNormal1,
  osPriority_t_osPriorityNormal2
};

type GpioTypeUserLed1 =  p_hal::gpio::gpioc::PC13<p_hal::gpio::Output<p_hal::gpio::PushPull>>;

// static USER_LED_1:  Mutex<RefCell<Option< GpioTypeUserLed1>>> = Mutex::new(RefCell::new(None));
static GLOBAL_QUEUE_HANDLE: AtomicPtr<osMessageQueueId_t> =  AtomicPtr::new(core::ptr::null_mut());
static UPDATE_COUNT: AtomicUsize = AtomicUsize::new(0);
static USER_LED1: AtomicPtr<GpioTypeUserLed1> =  AtomicPtr::new(core::ptr::null_mut());

// cortex-m-rt calls this for serious faults.  can set a breakpoint to debug
#[exception]
fn HardFault(_ef: &ExceptionFrame) -> ! {
  rprintln!("HardFault");
  loop {
    cortex_m::asm::bkpt();
  }
}

#[exception]
fn DefaultHandler(val: i16) -> ! {
  rprintln!("DefaultHandler {}", val);
  loop {
    //cortex_m::asm::bkpt();
  }
}

#[no_mangle]
extern "C" fn handle_assert_failed() {
  rprintln!("handle_assert_failed");
}



// Toggle the user leds from their prior state
fn toggle_leds() {
  unsafe {
    USER_LED1.load(Ordering::Relaxed).as_mut().unwrap().toggle().unwrap();
  }
}

fn task1_body(send_val: u8) -> i32 {
  let send_buf:[u8; 4] = [send_val, 0, 0, 0];
  let mq = GLOBAL_QUEUE_HANDLE.load(Ordering::Relaxed) as osMessageQueueId_t;
  let rc = cmsis_rtos2::rtos_os_msg_queue_put(
    mq,
    send_buf.as_ptr() as MsgBufType,
    1,
    250);

  rc
}

/// RTOS calls this function to run Task 1
#[no_mangle]
extern "C" fn task1_start(_arg: *mut cty::c_void) {
  // rprintln!("task1_start...");

  let mut send_val = 0;
  let exit_rc = loop {
    let rc = task1_body(send_val);
    send_val = send_val.wrapping_add(1);
    if rc != 0 {
      break rc;
    }
    cmsis_rtos2::rtos_os_thread_yield();
  };

  task1_done(exit_rc)

}

#[inline(never)]
fn task1_done(rc: i32) {
  rprintln!("task1 exit: {}", rc);
}

fn task2_body() -> i32 {
  let mut recv_buf: [u8; 4] = [0; 4];
  let rc = cmsis_rtos2::rtos_os_msg_queue_get(
    GLOBAL_QUEUE_HANDLE.load(Ordering::Relaxed) as osMessageQueueId_t,
    recv_buf.as_mut_ptr() as *mut cty::c_void,
    null_mut(), 250);
  if 0 == rc {
    toggle_leds();
    UPDATE_COUNT.fetch_add(1, Ordering::SeqCst);
  }

  rc
}

/// RTOS calls this function to run Task 2
#[no_mangle]
extern "C" fn task2_start(_arg: *mut cty::c_void) {

  // rprintln!("task2_start...");

  let exit_rc = loop {
    let rc = task2_body();
    cmsis_rtos2::rtos_os_thread_yield();

    if 0 != rc {
      break rc;
    }
  };

  task2_done(exit_rc)
}

#[inline(never)]
fn task2_done(rc: i32) {
  rprintln!("task2 exit: {}", rc);
}



pub fn setup_threads() {

// create a shared msg queue
  let mq = cmsis_rtos2::rtos_os_msg_queue_new(3, 4, null());
  if mq.is_null() {
   rprintln!("rtos_os_msg_queue_new failed");
   return;
  }
  GLOBAL_QUEUE_HANDLE.store(mq as *mut _, Ordering::Relaxed);

  let thread1_attr = thread_attr_with_priority(osPriority_t_osPriorityNormal);
  let thread1_id = cmsis_rtos2::rtos_os_thread_new(
    Some(task1_start),
    null_mut(),
    &thread1_attr,
  );
  if thread1_id.is_null() {
    rprintln!("rtos_os_thread1_new failed!");
    return;
  }

  let thread2_attr = thread_attr_with_priority(osPriority_t_osPriorityNormal);
  let thread2_id = cmsis_rtos2::rtos_os_thread_new(
    Some(task2_start),
    null_mut(),
    &thread2_attr,
  );
  if thread2_id.is_null() {
    rprintln!("rtos_os_thread2_new failed!");
    return;
  }


}


fn thread_attr_with_priority(priority: osPriority_t) -> osThreadAttr_t {
  osThreadAttr_t {
    name: null(),
    attr_bits: 0,
    cb_mem: null_mut(),
    cb_size: 0,
    stack_mem: null_mut(),
    stack_size: 0,
    priority,
    tz_module: 0,
    reserved: 0
  }
}

// Setup peripherals such as GPIO
fn setup_peripherals()  {
  rprint!( "setup_peripherals...");

  let dp = stm32::Peripherals::take().unwrap();

  let gpioc = dp.GPIOC.split();
  let mut user_led1 = gpioc.pc13.into_push_pull_output();

  // Set up the system clock at 16 MHz
  let rcc = dp.RCC.constrain();
  let _clocks = rcc.cfgr.freeze();

  //set initial states of user LEDs
  user_led1.set_high().unwrap();

  //store shared peripherals
  USER_LED1.store(&mut user_led1, Ordering::Relaxed);

  rprintln!("done!");
}


fn start_rtos() -> ! {
  rprintln!("Setup RTOS...");

  let _rc = cmsis_rtos2::rtos_kernel_initialize();
  let _tick_hz = cmsis_rtos2::rtos_kernel_get_tick_freq_hz();
  let _sys_timer_hz = cmsis_rtos2::rtos_kernel_get_sys_timer_freq_hz();
  rprintln!("tick_hz: {} sys_timer_hz: {} ", _tick_hz, _sys_timer_hz);

  setup_threads();

  // this should never return:
  let _rc = cmsis_rtos2::rtos_kernel_start();
  rprintln!("kernel exit: {}", _rc);

  unreachable!()
}

#[entry]
fn main() -> ! {
  rtt_init_print!(NoBlockSkip); //NoBlockTrim);
  rprintln!("-- MAIN --");
  
  setup_peripherals();
  // this should never return:
  start_rtos()

}
