#![no_std]

use embedded_hal::i2c::{I2c, SevenBitAddress};

const I2C_ADDR: u8 = 0x38;

const GEST_ID_ADDR: u8 = 0x01;
const TD_STATUS_ADDR: u8 = 0x02;
const CTRL_ADDR: u8 = 0x86;
const TIME_ENTER_MONITOR_ADDR: u8 = 0x87;
const PERIOD_ACTIVE_ADDR: u8 = 0x88;

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
    /// In monitor idle mode, the device will automatically switch to `monitor` mode after a timeout period of no touch events in active mode. This is the default mode.
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

/// An FT6x06 device.
pub struct FT6x06<I2C> {
    i2c: I2C,
}

impl<I2C: I2c<SevenBitAddress>> FT6x06<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self { i2c }
    }

    /// Retrieve the latest touch event with all available data and touch points.
    pub fn get_touch_event(&mut self) -> Result<Option<TouchEvent>, DriverError<I2C::Error>> {
        // Check if data is available.
        if !self.is_data_ready()? {
            return Ok(None);
        }

        // Read from 0x01 to 0x0E for GEST_ID, TD_STATUS, and PX_X registers.
        let mut buf = [0u8; 14];
        self.i2c.write_read(I2C_ADDR, &[GEST_ID_ADDR], &mut buf)?;

        // Get the number of touch points.
        // Less than 1 = no data; Greater than 2 = invalid.
        let num_touch_points = number_touch_points_from_register(buf[1]);
        if !is_num_touch_points_valid(num_touch_points) {
            return Ok(None);
        }

        let gesture = GestureType::from(buf[0]);

        // Get first touch point
        let primary_point =
            touch_point_from_registers(buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);

        // Get second touch point
        let mut secondary_point = None;
        if num_touch_points == 2 {
            let touch_point =
                touch_point_from_registers(buf[8], buf[9], buf[10], buf[11], buf[12], buf[13]);

            secondary_point = Some(touch_point);
        }

        Ok(Some(TouchEvent {
            primary_point,
            secondary_point,
            gesture,
        }))
    }

    /// Checks if there is touch data ready.
    pub fn is_data_ready(&mut self) -> Result<bool, DriverError<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(I2C_ADDR, &[TD_STATUS_ADDR], &mut buf)?;

        // We need only the last 4 bits.
        let num_touch_points = buf[0] & 0x0F;

        // Less than 1 = no data; Greater than 2 = invalid.
        if !is_num_touch_points_valid(num_touch_points) {
            return Ok(false);
        }

        Ok(true)
    }

    /// Get the current mode of the device.
    ///
    /// This corresponds to the `CTRL` register.
    pub fn get_control_mode(&mut self) -> Result<ControlMode, DriverError<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(I2C_ADDR, &[CTRL_ADDR], &mut buf)?;

        let value = ControlMode::try_from(buf[0]).map_err(|()| DriverError::InvalidResponse)?;
        Ok(value)
    }

    /// Set the device mode.
    ///
    /// The default control mode is [`ControlMode::MonitorIdle`].
    /// This corresponds to the `CTRL` register.
    pub fn set_control_mode(&mut self, mode: ControlMode) -> Result<(), DriverError<I2C::Error>> {
        Ok(self.i2c.write(I2C_ADDR, &[CTRL_ADDR, mode as u8])?)
    }

    /// Get the idle timeout for the [`ControlMode::MonitorIdle`] mode.
    ///
    /// This corresponds to the `TIMEENTERMONITOR` register.
    pub fn get_active_idle_timeout(&mut self) -> Result<u8, DriverError<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.i2c
            .write_read(I2C_ADDR, &[TIME_ENTER_MONITOR_ADDR], &mut buf)?;

        Ok(buf[0])
    }

    /// Set the idle timeout for the [`ControlMode::MonitorIdle`] mode.
    ///
    /// The default value is 10. The datasheet does not specify what type this value is (maybe seconds?).
    /// This corresponds to the `TIMEENTERMONITOR` register.
    pub fn set_active_idle_timeout(&mut self, timeout: u8) -> Result<(), DriverError<I2C::Error>> {
        Ok(self
            .i2c
            .write(I2C_ADDR, &[TIME_ENTER_MONITOR_ADDR, timeout])?)
    }

    /// Get the configured report rates for each mode.
    ///
    /// This corresponds to the `PERIODACTIVE` and `PERIODMONITOR` registers.
    pub fn get_report_rates(&mut self) -> Result<(u8, u8), I2C::Error> {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(I2C_ADDR, &[PERIOD_ACTIVE_ADDR], &mut buf)?;
        Ok((buf[0], buf[1]))
    }

    /// Set the configured report rates for each mode.
    ///
    /// The default report rates are:
    /// - 60hz for active mode.
    /// - 25hz for monitor mode.
    /// 
    /// The report rate is also called the scan rate.
    /// This corresponds to the `PERIODACTIVE` and `PERIODMONITOR` registers.
    pub fn set_report_rates(
        &mut self,
        active_rate: u8,
        monitor_rate: u8,
    ) -> Result<(), I2C::Error> {
        Ok(self
            .i2c
            .write(I2C_ADDR, &[PERIOD_ACTIVE_ADDR, active_rate, monitor_rate])?)
    }

    /// Safely clean up the device, returning any owned peripherals.
    pub fn destroy(self) -> I2C {
        self.i2c
    }
}

/// A driver error.
#[derive(Debug, defmt::Format)]
pub enum DriverError<I2CError> {
    /// A generic I2c communication error.
    I2cError(I2CError),
    /// The device returned something unexpected.
    InvalidResponse,
}

impl<I2CError> From<I2CError> for DriverError<I2CError> {
    fn from(value: I2CError) -> Self {
        Self::I2cError(value)
    }
}

/// Extract an entire [`TouchPoint`] from registers.
fn touch_point_from_registers(
    pn_xh: u8,
    pn_xl: u8,
    pn_yh: u8,
    pn_yl: u8,
    pn_weight: u8,
    pn_misc: u8,
) -> TouchPoint {
    let x = coord_from_registers(pn_xh, pn_xl);
    let y = coord_from_registers(pn_yh, pn_yl);
    let touch_id = touch_id_from_register(pn_yh);

    let touch_type = TouchType::from_register(pn_xh);
    let area = area_from_register(pn_misc);

    TouchPoint {
        x,
        y,
        weight: pn_weight,
        area,
        touch_type,
        touch_id,
    }
}

/// Returns true if the number of touch points is valid.
///
/// This will return false if there are no touch points or if the number of touch
/// points exceeds two.
fn is_num_touch_points_valid(amount: u8) -> bool {
    (1..=2).contains(&amount)
}

/// Extracts the 4 bit touch point amount from a register.
fn number_touch_points_from_register(value: u8) -> u8 {
    value & 0x0F
}

/// Extracts the 4 bit area from a register.
fn area_from_register(value: u8) -> u8 {
    value >> 4
}

/// Extracts the 4 bit touch id from a register.
fn touch_id_from_register(value: u8) -> u8 {
    value >> 4
}

/// Extracts the 12 bit coordinate from the msb and lsb registers.
fn coord_from_registers(msb: u8, lsb: u8) -> u16 {
    // last 4 bits are MSB
    let msb = msb & 0x0F;
    // Convert to 12 bit represented in u16.
    ((msb as u16) << 8) | lsb as u16
}
