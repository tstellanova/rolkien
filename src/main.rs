#![deny(warnings)]
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




use p_hal::rcc::Clocks;

use p_hal::gpio::GpioExt;
use p_hal::rcc::RccExt;

use core::ops::{DerefMut};

use p_hal::{prelude::*, stm32};
use core::ptr::{null, null_mut};
use cmsis_rtos2::{ osMessageQueueId_t};


type GpioTypeUserLed1 =  p_hal::gpio::gpioc::PC13<p_hal::gpio::Output<p_hal::gpio::PushPull>>;

static APP_CLOCKS:  Mutex<RefCell< Option< Clocks >>> = Mutex::new(RefCell::new(None));
static USER_LED_1:  Mutex<RefCell<Option< GpioTypeUserLed1>>> = Mutex::new(RefCell::new(None));
static mut GLOBAL_QUEUE_HANDLE: Option< osMessageQueueId_t  > = None;

// lazy_static {}
//   static ref GLOBAL_QUEUE_HANDLE: AtomicPtr<osMessageQueueId_t> = AtomicPtr::default();
//   // static mut GLOBAL_QUEUE_HANDLE: Option< osMessageQueueId_t  > = None;
// }



// cortex-m-rt calls this for serious faults.  can set a breakpoint to debug
#[exception]
fn HardFault(_ef: &ExceptionFrame) -> ! {
  loop {

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
    cmsis_rtos2::rtos_os_msg_queue_put(
      mq_id as osMessageQueueId_t,
      send_buf.as_ptr() as *const cty::c_void,
      1,
      250);

    send_buf[0] = (send_buf[0] + 1) % 255;
  }

}

/// RTOS calls this function to run Task 2
#[no_mangle]
extern "C" fn task2_cb(_arg: *mut cty::c_void) {
  let mq_id:osMessageQueueId_t = unsafe { GLOBAL_QUEUE_HANDLE.unwrap() } ;
  let mut recv_buf: [u8; 10] = [0; 10];

  loop {
    let rc = cmsis_rtos2::rtos_os_msg_queue_get(mq_id,
                                                recv_buf.as_mut_ptr() as *mut cty::c_void,
                                                null_mut(), 100);
    if 0 == rc {
      toggle_leds();
      cmsis_rtos2::rtos_os_delay(50);
    }

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
    rprintln!("rtos_os_thread_new failed!");
    return;
  }

  let thread2_id = cmsis_rtos2::rtos_os_thread_new(
    Some(task2_cb),
    null_mut(),
    null(),
  );
  if thread2_id.is_null() {
    rprintln!("rtos_os_thread_new failed!");
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
  let clocks = rcc.cfgr.freeze();
//  let clocks = rcc.cfgr.sysclk(16.mhz()).freeze();

  //set initial states of user LEDs
  user_led1.set_high().unwrap();

  //store shared peripherals
  interrupt::free(|cs| {
    APP_CLOCKS.borrow(cs).replace(Some(clocks));
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
