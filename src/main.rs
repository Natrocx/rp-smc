#![no_std]
#![no_main]
mod consts;

use panic_halt as _;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {

  use defmt::*;
  use defmt_rtt as _;
  use embedded_hal::digital::v2::{InputPin, OutputPin};
  use rp_pico::hal::clocks::init_clocks_and_plls;
  use rp_pico::hal::timer::Alarm;
  use rp_pico::hal::watchdog::Watchdog;
  use rp_pico::hal::{self, Sio};
  use rp_pico::XOSC_CRYSTAL_FREQ;

  use crate::consts::*;

  #[shared]
  struct Shared {
    alarm: hal::timer::Alarm0,
    short_pin: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio18, hal::gpio::PushPullOutput>,
  }

  #[local]
  struct Local {
    led: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio25, hal::gpio::PushPullOutput>,
    continuous_pin: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio19, hal::gpio::PushPullOutput>,
    interrupt_pin: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio20, hal::gpio::PullDownInput>,
    timer: hal::Timer,
  }

  #[init]
  fn init(mut c: init::Context) -> (Shared, Local, init::Monotonics) {
    // Soft-reset does not release the hardware spinlocks
    // Release them now to avoid a deadlock after debug or watchdog reset
    unsafe {
      hal::sio::spinlock_reset();
    }
    info!("Initializing Raspberry Pi Pico.");

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
      Shared { alarm, short_pin },
      Local {
        led,
        timer,
        interrupt_pin,
        continuous_pin,
      },
      init::Monotonics(),
    )
  }

  #[task(
        binds = TIMER_IRQ_0,
        priority = 2,
        shared = [alarm, short_pin],
    )]
  fn timer_irq(c: timer_irq::Context) {
    info!("From Timer interrupt.");

    (c.shared.alarm, c.shared.short_pin).lock(|alarm, short_pin| {
      alarm.clear_interrupt();
      short_pin.set_state(!GPIO_SHORT_ACTUATION_ACTIVE_POLARITY).unwrap();
    });
  }

  #[task(
        binds = IO_IRQ_BANK0,
        priority = 1,
        shared = [alarm, short_pin],
        local = [power: bool = false,
                 edge_time: u64 = 0,
                 polling: bool = false, // guards edge time value
                 has_toggled: bool = false, // guards PSU power toggle/active write
                 led, continuous_pin, interrupt_pin, timer]
    )]
  fn gpio_irq(c: gpio_irq::Context) {
    // Algorithm:
    // `
    // let local_resource polling = false;
    //
    // if interrupt pin inactive {
    //   encountered falling edge => disable interrupt; quit;
    //   polling = false
    // }
    // else:
    //
    // let ct = current time;
    // let fe = first edge time;
    //
    // if encountered rising edge (equivalent to polling == false) {
    //   fe = now;
    //   polling = true;
    // }
    //
    // if ct - fe > tolerance time  && polling {
    //   accept button press
    // }
    // else {
    //   button was bouncy
    // }
    // `
    let current_time = c.local.timer.get_counter();
    debug!(
      "GPIO IRQ at power({}): {}, {}",
      c.local.power, current_time, c.local.edge_time
    );

    if !*c.local.polling {
      *c.local.edge_time = current_time;
      *c.local.polling = true;
    }

    if (current_time - *c.local.edge_time) as u32 > BUTTON_TOLERANCE && !*c.local.has_toggled {
      *c.local.power = !*c.local.power;
      *c.local.has_toggled = true;
      *c.local.polling = false;
      let (continuous_power, short_power) = match *c.local.power {
        true => (
          GPIO_CONTINUOUS_ACTUATION_ACTIVE_POLARITY,
          GPIO_SHORT_ACTUATION_ACTIVE_POLARITY,
        ),
        false => (
          !GPIO_CONTINUOUS_ACTUATION_ACTIVE_POLARITY,
          !GPIO_SHORT_ACTUATION_ACTIVE_POLARITY,
        ),
      };

      (c.shared.short_pin, c.shared.alarm).lock(|sp, alarm| {
        c.local.continuous_pin.set_state(continuous_power).unwrap();
        sp.set_state(short_power).unwrap();

        if cfg!(not(feature = "wifi")) {
          c.local.led.set_state((*c.local.power).into()).unwrap();
        }

        alarm
          .schedule(fugit::Duration::<u32, 1, 1000000>::micros(GPIO_SHORT_ACTUATION_PERIOD))
          .unwrap();

        info!("Button press detected.");
      });
    }

    if c.local.interrupt_pin.is_low().unwrap() {
      c.local.interrupt_pin.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
      *c.local.polling = false;
      *c.local.has_toggled = false;
    }
  }

  #[idle]
  fn idle(_c: idle::Context) -> ! {
    loop {
      // low power idle
      cortex_m::asm::nop();
    }
  }
}
