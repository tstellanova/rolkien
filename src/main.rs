// #![deny(warnings)]
#![no_main]
#![no_std]


use panic_rtt_core::{self, rprintln, rtt_init_print};

use core::cell::RefCell;
use cortex_m::interrupt::{self, Mutex};

use stm32f4xx_hal as p_hal;

use cortex_m_rt::{entry, exception, ExceptionFrame};

use p_hal::hal::digital::v2::ToggleableOutputPin;

use freertos_sys::cmsis_rtos2;

#[allow(non_upper_case_globals)]
#[no_mangle]
pub static SystemCoreClock: u32 = 16_000_000; //or use stm32f4xx_hal rcc::HSI
//Can use 25_000_000 on an stm32f401 board with 25 MHz xtal
// 48_000_000 for stm32h743 HSI (48 MHz)



use p_hal::gpio::GpioExt;
use p_hal::rcc::RccExt;

use core::ops::{DerefMut};
use core::sync::atomic::{AtomicU32, Ordering};

use p_hal::{prelude::*, stm32};
use core::ptr::{null, null_mut};
use cmsis_rtos2::{ osMessageQueueId_t, osThreadAttr_t, osPriority_t_osPriorityLow};


type GpioTypeUserLed1 =  p_hal::gpio::gpioc::PC13<p_hal::gpio::Output<p_hal::gpio::PushPull>>;

static USER_LED_1:  Mutex<RefCell<Option< GpioTypeUserLed1>>> = Mutex::new(RefCell::new(None));
static mut GLOBAL_QUEUE_HANDLE: Option< osMessageQueueId_t  > = None;
static UPDATE_COUNT: AtomicU32 = AtomicU32::new(0);

// lazy_static {}
//   static ref GLOBAL_QUEUE_HANDLE: AtomicPtr<osMessageQueueId_t> = AtomicPtr::default();
//   // static mut GLOBAL_QUEUE_HANDLE: Option< osMessageQueueId_t  > = None;
// }



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
  interrupt::free(|cs| {
    if let Some(ref mut led1) = USER_LED_1.borrow(cs).borrow_mut().deref_mut() {
      led1.toggle().unwrap();
    }
  });
}

/// RTOS calls this function to run Task 1
#[no_mangle]
extern "C" fn task1_cb(_arg: *mut cty::c_void) {
  let mq_id:osMessageQueueId_t = unsafe { GLOBAL_QUEUE_HANDLE.unwrap() } ;
  let mut send_buf: [u8; 10] = [0; 10];
  loop {
    let rc = cmsis_rtos2::rtos_os_msg_queue_put(
      mq_id as osMessageQueueId_t,
      send_buf.as_ptr() as *const cty::c_void,
      1,
      250);

    if 0 != rc {
      rprintln!("qput failed: {}", rc);
    }
    send_buf[0] =  send_buf[0].wrapping_add(1);
    cmsis_rtos2::rtos_os_thread_yield();
  }

}

/// RTOS calls this function to run Task 2
#[no_mangle]
extern "C" fn task2_cb(_arg: *mut cty::c_void) {
  let mut recv_buf: [u8; 10] = [0; 10];

  loop {
    let rc = cmsis_rtos2::rtos_os_msg_queue_get(
      unsafe { GLOBAL_QUEUE_HANDLE.unwrap() },
      recv_buf.as_mut_ptr() as *mut cty::c_void,
      null_mut(), 250);
    if 0 == rc {
      toggle_leds();
      UPDATE_COUNT.fetch_add(1, Ordering::Relaxed);
    }
    else {
      rprintln!("get fail: {}", rc);
    }
    cmsis_rtos2::rtos_os_delay(50);

  }

}

/// RTOS calls this function to run Task 3
#[no_mangle]
extern "C" fn task3_cb(_arg: *mut cty::c_void) {

  loop {
    let count_val = UPDATE_COUNT.load(Ordering::Relaxed);
    rprintln!("count {}", count_val);
    cmsis_rtos2::rtos_os_delay(1000);
  }


}



pub fn setup_default_threads() {

// create a shared msg queue
  let mq = cmsis_rtos2::rtos_os_msg_queue_new(10, 4, null());
  if mq.is_null() {
   rprintln!("rtos_os_msg_queue_new failed");
   return;
  }

  unsafe {
   GLOBAL_QUEUE_HANDLE = Some(mq);
  }

  // We don't pass context to the default task here, since that involves problematic
  // casting to/from C void pointers; instead, we use global static context.
  let thread1_id = cmsis_rtos2::rtos_os_thread_new(
    Some(task1_cb),
    null_mut(),
    null(),
  );
  if thread1_id.is_null() {
    rprintln!("rtos_os_thread1_new failed!");
    return;
  }

  let thread2_id = cmsis_rtos2::rtos_os_thread_new(
    Some(task2_cb),
    null_mut(),
    null(),
  );
  if thread2_id.is_null() {
    rprintln!("rtos_os_thread2_new failed!");
    return;
  }


  let attr = osThreadAttr_t {
    name: null(),
    attr_bits: 0,
    cb_mem: null_mut(),
    cb_size: 0,
    stack_mem: null_mut(),
    stack_size: 0,
    priority: osPriority_t_osPriorityLow,
    tz_module: 0,
    reserved: 0
  };
  
  let thread3_id = cmsis_rtos2::rtos_os_thread_new(
    Some(task3_cb),
    null_mut(),
    &attr,
  );
  if thread3_id.is_null() {
    rprintln!("rtos_os_thread3_new failed!");
    return;
  }

}

// Setup peripherals such as GPIO
fn setup_peripherals()  {
  //rprintln!(, "setup_peripherals...");

  let dp = stm32::Peripherals::take().unwrap();

  let gpioc = dp.GPIOC.split();
  let mut user_led1 = gpioc.pc13.into_push_pull_output();

  // Set up the system clock at 16 MHz
  let rcc = dp.RCC.constrain();
  let _clocks = rcc.cfgr.freeze();
//  let clocks = rcc.cfgr.sysclk(16.mhz()).freeze();

  //set initial states of user LEDs
  user_led1.set_high().unwrap();

  //store shared peripherals
  interrupt::free(|cs| {
    USER_LED_1.borrow(cs).replace(Some(user_led1));
  });

  //rprintln!(, "done!");

}


fn setup_rtos() {
//  rprintln!(, "Setup RTOS...");

  let _rc = cmsis_rtos2::rtos_kernel_initialize();
  let _tick_hz = cmsis_rtos2::rtos_kernel_get_tick_freq_hz();
  let _sys_timer_hz = cmsis_rtos2::rtos_kernel_get_sys_timer_freq_hz();

//  setup_repeating_timer();
  setup_default_threads();

  // this should never return:
  let _rc = cmsis_rtos2::rtos_kernel_start();

}

#[entry]
fn main() -> ! {
  rtt_init_print!(NoBlockTrim);
  rprintln!("-- > MAIN --");
  
  setup_peripherals();
  setup_rtos();

  loop {
    //cmsis_rtos2::rtos_os_thread_yield();
    //one hz heartbeat
    cmsis_rtos2::rtos_os_delay(1000);
    rprintln!(".");
  }

}
