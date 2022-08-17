#![no_std]
#![no_main]
mod consts;

use panic_halt as _;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {

  use cortex_m::delay::Delay;
use defmt::*;
  use defmt_rtt as _;
  use embedded_hal::digital::v2::{OutputPin, PinState};
  use embedded_time::duration::Extensions;
  use rp_pico::hal::clocks::init_clocks_and_plls;
  use rp_pico::hal::timer::Alarm;
  use rp_pico::hal::watchdog::Watchdog;
  use rp_pico::hal::{self, Sio};
  use rp_pico::XOSC_CRYSTAL_FREQ;

use crate::consts::TIMER_FREQUENCY;


  const SCAN_TIME_US: u32 = 1000000;

  #[shared]
  struct Shared {
    timer: hal::Timer,
    alarm: hal::timer::Alarm0,
    led: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio25, hal::gpio::PushPullOutput>,
    interrupt_pin: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio20, hal::gpio::PullDownInput>,
  }

  #[local]
  struct Local {
    delay: Delay
  }

  #[init]
  fn init(mut c: init::Context) -> (Shared, Local, init::Monotonics) {
    info!("Initializing rpi");
    // Soft-reset does not release the hardware spinlocks
    // Release them now to avoid a deadlock after debug or watchdog reset
    unsafe {
      hal::sio::spinlock_reset();
    }
    let mut resets = c.device.RESETS;
    let mut watchdog = Watchdog::new(c.device.WATCHDOG);
    let clocks = init_clocks_and_plls(
      XOSC_CRYSTAL_FREQ,
      c.device.XOSC,
      c.device.CLOCKS,
      c.device.PLL_SYS,
      c.device.PLL_USB,
      &mut resets,
      &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = Sio::new(c.device.SIO);
    let pins = rp_pico::Pins::new(c.device.IO_BANK0, c.device.PADS_BANK0, sio.gpio_bank0, &mut resets);
    let mut led = pins.led.into_push_pull_output();
    led.set_high().unwrap();

    let interrupt_pin = pins.gpio20.into_pull_down_input();
    interrupt_pin.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);

    unsafe { rp_pico::pac::NVIC::unmask(rp_pico::pac::Interrupt::IO_IRQ_BANK0) }

    let mut timer = hal::Timer::new(c.device.TIMER, &mut resets);
    let mut alarm = timer.alarm_0().unwrap();
    // let _ = alarm.schedule(SCAN_TIME_US.microseconds());
    alarm.enable_interrupt();

    let delay = Delay::new(c.core.SYST, TIMER_FREQUENCY as u32);

    c.core.SCB.set_sleepdeep();


    info!("Initialization successful, delegating control flow to rtic");

    (
      Shared {
        timer,
        alarm,
        led,
        interrupt_pin,
      },
      Local {delay},
      init::Monotonics(),
    )
  }

  #[task(
        binds = TIMER_IRQ_0,
        priority = 2,
        shared = [timer, alarm, led],
    )]
  fn timer_irq(mut c: timer_irq::Context) {
    info!("From Timer interrupt.");

    (&mut c.shared.led, &mut c.shared.alarm).lock(|l, alarm| {
      l.set_low().unwrap();
      alarm.clear_interrupt();
    });
  }

  #[task(
        binds = IO_IRQ_BANK0,
        priority = 1,
        shared = [alarm, led, interrupt_pin, timer],
        local = [led_active: PinState = PinState::Low, last_value: u64 = 0]
    )]
  fn gpio_irq(mut c: gpio_irq::Context) {
    debug!("Entering GPIO IRQ with local state: {}", *c.local.last_value);
    *c.local.led_active = !*c.local.led_active;

    // write state using single lock
    (
      &mut c.shared.led,
      &mut c.shared.interrupt_pin,
      &mut c.shared.alarm,
      &mut c.shared.timer,
    )
      .lock(|led, interrupt_pin, alarm, timer| {
        if (*c.local.last_value + TIMER_FREQUENCY ) < timer.get_counter() {
          *c.local.last_value = timer.get_counter();
          led.set_state(*c.local.led_active).unwrap();
          alarm.schedule(SCAN_TIME_US.microseconds()).unwrap();
          info!("Button press detected.");
        }
        interrupt_pin.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
      });
  }

  #[idle(shared = [timer], local = [delay])]
  fn idle(_c: idle::Context) -> ! {
    loop {
     cortex_m::asm::nop();
     }
  }
}
