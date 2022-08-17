#![no_std]
#![no_main]
mod consts;

use panic_halt as _;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {

  use defmt::*;
  use defmt_rtt as _;
  use embedded_hal::digital::v2::OutputPin;
  use embedded_time::duration::Extensions;
  use rp_pico::hal::clocks::init_clocks_and_plls;
  use rp_pico::hal::timer::Alarm;
  use rp_pico::hal::watchdog::Watchdog;
  use rp_pico::hal::{self, Sio};
  use rp_pico::XOSC_CRYSTAL_FREQ;

  use crate::consts::{self, GPIO_CONTINUOUS_ACTUATION_ACTIVE_POLARITY, GPIO_SHORT_ACTUATION_PERIOD, TIMER_FREQUENCY};

  #[shared]
  struct Shared {
    timer: hal::Timer,
    alarm: hal::timer::Alarm0,
    interrupt_pin: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio20, hal::gpio::PullDownInput>,
    continuous_pin: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio19, hal::gpio::PushPullOutput>,
    short_pin: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio18, hal::gpio::PushPullOutput>,
  }

  #[local]
  struct Local {
    led: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio25, hal::gpio::PushPullOutput>,
  }

  #[init]
  fn init(mut c: init::Context) -> (Shared, Local, init::Monotonics) {
    info!("Initializing Raspberry Pi Pico.");
    // Soft-reset does not release the hardware spinlocks
    // Release them now to avoid a deadlock after debug or watchdog reset
    unsafe {
      hal::sio::spinlock_reset();
    }

    let mut resets = c.device.RESETS;
    let mut watchdog = Watchdog::new(c.device.WATCHDOG);
    let _clocks = init_clocks_and_plls(
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
    if cfg!(feature = "wifi") {
      led.set_high().unwrap();
    }

    let interrupt_pin = pins.gpio20.into_pull_down_input();
    interrupt_pin.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);

    let short_pin = pins.gpio18.into_push_pull_output();
    let continuous_pin = pins.gpio19.into_push_pull_output();

    let mut timer = hal::Timer::new(c.device.TIMER, &mut resets);
    let mut alarm = timer.alarm_0().unwrap();
    alarm.enable_interrupt();

    // Configure cores
    c.core.SCB.set_sleepdeep();
    unsafe { rp_pico::pac::NVIC::unmask(rp_pico::pac::Interrupt::IO_IRQ_BANK0) }

    info!("Initialization successful, delegating control flow to rtic");

    (
      Shared {
        timer,
        alarm,
        interrupt_pin,
        continuous_pin,
        short_pin,
      },
      Local { led },
      init::Monotonics(),
    )
  }

  #[task(
        binds = TIMER_IRQ_0,
        priority = 2,
        shared = [timer, alarm],
    )]
  fn timer_irq(mut c: timer_irq::Context) {
    info!("From Timer interrupt.");

    c.shared.alarm.lock(|alarm| {
      alarm.clear_interrupt();
    });
  }

  #[task(
        binds = IO_IRQ_BANK0,
        priority = 1,
        shared = [alarm, interrupt_pin, timer, short_pin, continuous_pin],
        local = [power: bool = false, last_value: u64 = 0, led]
    )]
  fn gpio_irq(mut c: gpio_irq::Context) {
    debug!(
      "Entering GPIO IRQ with local state: {}, {}",
      *c.local.last_value, *c.local.power
    );

    // toggle state
    *c.local.power = !*c.local.power;

    let (continuous_power, short_power) = match *c.local.power {
      true => (
        GPIO_CONTINUOUS_ACTUATION_ACTIVE_POLARITY,
        consts::GPIO_SHORT_ACTUATION_ACTIVE_POLARITY,
      ),
      false => (
        !GPIO_CONTINUOUS_ACTUATION_ACTIVE_POLARITY,
        !consts::GPIO_SHORT_ACTUATION_ACTIVE_POLARITY,
      ),
    };

    // write state using single lock
    (
      &mut c.shared.interrupt_pin,
      &mut c.shared.short_pin,
      &mut c.shared.continuous_pin,
      &mut c.shared.alarm,
      &mut c.shared.timer,
    )
      .lock(|ip, sp, cp, alarm, timer| {
        if (*c.local.last_value + TIMER_FREQUENCY) < timer.get_counter() {
          cp.set_state(continuous_power).unwrap();
          sp.set_state(short_power).unwrap();

          if cfg!(not(feature = "wifi")) {
            c.local.led.set_state((*c.local.power).into()).unwrap();
          }

          *c.local.last_value = timer.get_counter();
          alarm.schedule(GPIO_SHORT_ACTUATION_PERIOD.microseconds()).unwrap();

          info!("Button press detected.");
        }
        ip.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
      });
  }

  #[idle(shared = [timer])]
  fn idle(_c: idle::Context) -> ! {
    loop {
      cortex_m::asm::nop();
    }
  }
}
