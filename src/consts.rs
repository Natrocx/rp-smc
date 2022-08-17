// Definitions valid for one particular electrical setup

use embedded_hal::digital::v2::PinState;
/// Whether to drive the Pin to high or low voltage.
///
/// # Semantics
/// If this is set to true, the Pin will be driven to 3.3V to activate the connected device,
/// otherwise, it will be driven to Ground.
pub const GPIO_SHORT_ACTUATION_ACTIVE_POLARITY: PinState = PinState::High;

/// The rp2040 comes with a 1 MHz Timer, and 64 bits precision. Overflows need not be considered.
pub const TIMER_FREQUENCY: u64 = 1_000_000;