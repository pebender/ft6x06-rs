use embedded_hal::i2c::{I2c, SevenBitAddress};

use crate::{
    common::{
        self, CTRL_ADDR, GEST_ID_ADDR, G_MODE_ADDR, I2C_ADDR, PERIOD_ACTIVE_ADDR,
        TIME_ENTER_MONITOR_ADDR,
    },
    ControlMode, DriverError, InterruptMode, TouchEvent,
};

/// An FT6x06 device.
///
/// This struct contains the full sync interface to the FT6x06 device.
/// Certain features depend upon the specific FT6x06 chip and firmware version and some data or functionality may be unavailable.
pub struct FT6x06<I2C> {
    i2c: I2C,
}

impl<I2C: I2c<SevenBitAddress>> FT6x06<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self { i2c }
    }

    /// Retrieve the latest touch event with all available data and touch points.
    pub fn get_touch_event(&mut self) -> Result<Option<TouchEvent>, DriverError<I2C::Error>> {
        // Read from 0x01 to 0x0E for GEST_ID, TD_STATUS, and PX_X registers.
        let mut buf = [0u8; 14];
        self.i2c.write_read(I2C_ADDR, &[GEST_ID_ADDR], &mut buf)?;
        Ok(common::touch_event_from_buf(&buf))
    }

    /// Get the current mode of the device.
    /// This corresponds to the `CTRL` register.
    pub fn get_control_mode(&mut self) -> Result<ControlMode, DriverError<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(I2C_ADDR, &[CTRL_ADDR], &mut buf)?;

        let value = ControlMode::try_from(buf[0]).map_err(|()| DriverError::InvalidResponse)?;
        Ok(value)
    }

    /// Set the device mode.
    /// The default control mode is [`ControlMode::MonitorIdle`].
    /// This corresponds to the `CTRL` register.
    pub fn set_control_mode(&mut self, mode: ControlMode) -> Result<(), DriverError<I2C::Error>> {
        Ok(self.i2c.write(I2C_ADDR, &[CTRL_ADDR, mode as u8])?)
    }

    /// Get the idle timeout for the [`ControlMode::MonitorIdle`] mode.
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
    /// This corresponds to the `PERIODACTIVE` and `PERIODMONITOR` registers.
    pub fn get_report_rates(&mut self) -> Result<(u8, u8), DriverError<I2C::Error>> {
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
    ) -> Result<(), DriverError<I2C::Error>> {
        Ok(self
            .i2c
            .write(I2C_ADDR, &[PERIOD_ACTIVE_ADDR, active_rate, monitor_rate])?)
    }

    /// Get the current interrupt mode.
    /// This corresponds to the `G_MODE` register.
    pub fn get_interrupt_mode(&mut self) -> Result<InterruptMode, DriverError<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(I2C_ADDR, &[G_MODE_ADDR], &mut buf)?;

        let value = InterruptMode::try_from(buf[0]).map_err(|()| DriverError::InvalidResponse)?;
        Ok(value)
    }

    /// Set the current interrupt mode.
    /// This corresponds to the `G_MODE` register.
    pub fn set_interrupt_mode(
        &mut self,
        mode: InterruptMode,
    ) -> Result<(), DriverError<I2C::Error>> {
        Ok(self.i2c.write(I2C_ADDR, &[G_MODE_ADDR, mode as u8])?)
    }

    /// Read a byte from any register.
    /// Provided as a catch-all for missing api.
    pub fn read_register(&mut self, reg: u8) -> Result<u8, DriverError<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(I2C_ADDR, &[reg], &mut buf)?;
        Ok(buf[0])
    }

    /// Read multiple bytes from any register. Provided as a catch-all for
    /// missing api.
    /// 
    /// For at least some hardware/firmware compbinations, the register 0xD3 can
    /// be read to retrieve the Gesture Id, the number of points used to
    /// determine the gesture and the points used to determine the gesture.
    pub fn read_register_multivalue(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), DriverError<I2C::Error>> {
        self.i2c.write_read(I2C_ADDR, &[reg], buf)?;
        Ok(())
    }

    /// Write a byte to any register.
    /// Provided as a catch-all for missing api.
    ///
    /// # Safety
    /// This function allows you to write any value to any register and as such is marked unsafe.
    pub unsafe fn write_register(
        &mut self,
        reg: u8,
        val: u8,
    ) -> Result<(), DriverError<I2C::Error>> {
        Ok(self.i2c.write(I2C_ADDR, &[reg, val])?)
    }

    /// Safely clean up the device, returning any owned peripherals.
    pub fn destroy(self) -> I2C {
        self.i2c
    }
}
