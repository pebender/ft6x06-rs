//! `ft6x06-rs` is a pure-Rust `embedded-hal`-based driver for the I2C-based `ft6x06` capacitive touch
//! screen controller. This crate aims to provide high-level functionality for most use-cases.
//!
//! All interactions are through the `FT6x06` or `FT6x06Async` struct depending on the enabled features.
//!
//! **Features**
//! - `sync-driver` - default, sync driver using `embedded-hal`.
//! - `async-driver` - async version of the driver using `embedded-hal-async`.
//!
//! The sync and async drivers are the same except async includes `wait_for_touch` functionality.
//!
//! **Example**
//! ```rust
//! // Initialization
//! let mut dev = FT6x06::new(i2c);
//!
//! // Configure the device.
//! dev.set_interrupt_mode(InterruptMode::Trigger)?;
//! dev.set_control_mode(ControlMode::MonitorIdle)?;
//! dev.set_active_idle_timeout(10)?;
//! dev.set_report_rates(60, 25)?;
//!
//! // Read the device configuration.
//! let interrupt_mode = dev.get_interrupt_mode()?;
//! let control_mode = dev.get_control_mode()?;
//! let active_idle_timeout = dev.get_active_idle_timeout()?;
//! let (active_rate, monitor_rate) = dev.get_report_rates()?;
//!
//! info!("Irq Mode: {}", interrupt_mode);
//! info!("Ctrl Mode: {}", control_mode);
//! info!("Active Idle Timeout: {}", active_idle_timeout);
//! info!("Active Rate: {}", active_rate);
//! info!("Monitor Rate: {}", monitor_rate);
//!
//! // Get the latest touch data. Usually after receiving an interrupt from the device.
//! let touch_event = dev.get_touch_event()?;
//!
//! // In the async driver, you can wait for the next touch event:
//! let mut dev_async = FT6x06Async::new(i2c).with_irq_pin(my_periph.GPIO_XX);
//! let touch_event = dev_async.wait_for_touch().await?;
//! ```

#![no_std]

mod common;

// Sync imports
#[cfg(feature = "sync-driver")]
mod sync;
#[cfg(feature = "sync-driver")]
pub use sync::FT6x06;

// Async imports
#[cfg(feature = "async-driver")]
mod asynch;
#[cfg(feature = "async-driver")]
pub use asynch::FT6x06Async;

/// A recorded touch event.
///
/// Some data may not be available depending on your hardware.
/// For example, the [AdaFruit product 2090](https://www.adafruit.com/product/2090) only supports a
/// single [`TouchPoint`] with position and [`TouchType`].
#[derive(Debug, defmt::Format, PartialEq, Clone)]
pub struct TouchEvent {
    /// The 1st recorded touch point in this event.
    pub primary_point: TouchPoint,
    /// The 2nd recorded touch point in this event.
    pub secondary_point: Option<TouchPoint>,
    /// The gesture type determined by the sensor.
    pub gesture: GestureType,
}

/// A recorded touch point.
#[derive(Debug, defmt::Format, PartialEq, Clone)]
pub struct TouchPoint {
    /// The x position of the touch event. Recorded at 12 bits.
    pub x: u16,
    /// The y position of the touch event. Recorded at 12 bits.
    pub y: u16,
    /// The pressure weight of this touch event.
    pub weight: u8,
    /// The size of this touch event. Recorded at 4 bits.
    ///
    /// This can help determine the weight of the touch as harder presses will cause more surface area of a finger to touch the screen.
    pub area: u8,
    /// The type of touch that occured for this touch point.
    pub touch_type: TouchType,
    /// The sensor ID for the touch point.
    pub touch_id: u8,
}

/// The type of touch for a touch point.
#[derive(Debug, defmt::Format, PartialEq, Clone, Copy)]
pub enum TouchType {
    PressDown,
    LiftUp,
    Contact,
    NoEvent,
    Invalid,
}

impl TouchType {
    /// Gets the TouchType from the first two event flag bits in a byte.
    pub fn from_register(reg: u8) -> Self {
        let data = reg >> 6;
        Self::from(data)
    }
}

impl From<u8> for TouchType {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::PressDown,
            1 => Self::LiftUp,
            2 => Self::Contact,
            3 => Self::NoEvent,
            _ => Self::Invalid,
        }
    }
}

/// The type of gesture detected for a touch event.
#[derive(Debug, defmt::Format, PartialEq, Clone, Copy)]
pub enum GestureType {
    MoveUp,
    MoveRight,
    MoveDown,
    MoveLeft,
    ZoomIn,
    ZoomOut,
    NoGesture,
    Invalid,
}

impl From<u8> for GestureType {
    fn from(value: u8) -> Self {
        match value {
            0x10 => Self::MoveUp,
            0x14 => Self::MoveRight,
            0x18 => Self::MoveDown,
            0x1C => Self::MoveLeft,
            0x48 => Self::ZoomIn,
            0x49 => Self::ZoomOut,
            0x00 => Self::NoGesture,
            _ => Self::Invalid,
        }
    }
}

/// The mode that the device will use to scan for touch events.
///
/// This corresponds to the `CTRL` register.
#[derive(Debug, defmt::Format, PartialEq, Clone, Copy)]
pub enum ControlMode {
    /// Force active will force the device to continously scan at it's full rate.
    ForceActive = 0,
    /// In monitor idle mode, the device will automatically switch to `monitor` mode after a timeout period of no touch events in active mode.
    ///
    /// The timeout period can be set with [`FT6x06::set_active_idle_timeout`].
    MonitorIdle = 1,
}

impl TryFrom<u8> for ControlMode {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(ControlMode::ForceActive),
            1 => Ok(ControlMode::MonitorIdle),
            _ => Err(()),
        }
    }
}

/// The interrupt mode.
///
/// This corresponds to the `G_MODE` register.
#[derive(Debug, defmt::Format, PartialEq, Clone, Copy)]
pub enum InterruptMode {
    /// Interrupt poll mode will pull the interrupt pin low until every touch event ends.
    Poll = 0,
    /// Interrupt trigger mode will cycle the interrupt pin for every touch event.
    Trigger = 1,
}

impl TryFrom<u8> for InterruptMode {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Poll),
            1 => Ok(Self::Trigger),
            _ => Err(()),
        }
    }
}

/// A driver error.
#[derive(Debug, defmt::Format)]
pub enum DriverError<I2CError> {
    /// A generic I2c communication error.
    I2cError(I2CError),
    /// The device returned something unexpected.
    InvalidResponse,
    /// Returned by the async driver if the irq pin wasn't set for the required functionality.
    IrqPinNotSet,
    /// Returned when an error occured while waiting for an IRQ.
    IrqError,
}

impl<I2CError> From<I2CError> for DriverError<I2CError> {
    fn from(value: I2CError) -> Self {
        Self::I2cError(value)
    }
}
