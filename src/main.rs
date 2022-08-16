#![no_std]
#![no_main]

use panic_halt as _;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {

  use defmt::*;
  use defmt_rtt as _;
  use embedded_hal::digital::v2::{OutputPin, PinState};
  use embedded_time::duration::Extensions;
  use rp_pico::hal::clocks::init_clocks_and_plls;
  use rp_pico::hal::timer::Alarm;
  use rp_pico::hal::watchdog::Watchdog;
  use rp_pico::hal::{self, Sio};
  use rp_pico::XOSC_CRYSTAL_FREQ;

  const SCAN_TIME_US: u32 = 1000000;

  #[shared]
  struct Shared {
    timer: hal::Timer,
    alarm: hal::timer::Alarm0,
    led: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio25, hal::gpio::PushPullOutput>,
    interrupt_pin: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio20, hal::gpio::PullDownInput>,
  }

  #[local]
  struct Local {}

  #[init]
  fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
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
    /*
    // Set up the USB driver
    let usb_bus = usb_device::class_prelude::UsbBusAllocator::new(hal::usb::UsbBus::new(
      c.device.USBCTRL_REGS,
      c.device.USBCTRL_DPRAM,
      clocks.usb_clock,
      true,
      &mut resets,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = usbd_serial::SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev =
      usb_device::prelude::UsbDeviceBuilder::new(&usb_bus, usb_device::prelude::UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Natrocx autism Ltd.")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();
    */

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

    info!("Initialization successful, relegating control flow to rtic");

    (
      Shared {
        timer,
        alarm,
        led,
        interrupt_pin,
      },
      Local {},
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
        shared = [alarm, led, interrupt_pin],
        local = [led_active: PinState = PinState::Low]
    )]
  fn gpio_irq(mut c: gpio_irq::Context) {
    info!("From GPIO interrupt.");
    *c.local.led_active = !*c.local.led_active;

    // write state using single lock
    (&mut c.shared.led, &mut c.shared.interrupt_pin, &mut c.shared.alarm).lock(|led, interrupt_pin, alarm| {
      led.set_state(*c.local.led_active).unwrap();
      interrupt_pin.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
      alarm.schedule(SCAN_TIME_US.microseconds()).unwrap();
    });
  }
}
