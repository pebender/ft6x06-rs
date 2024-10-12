use embedded_hal_async::{
    digital::Wait,
    i2c::{I2c, SevenBitAddress},
};

use crate::{
    common::{
        self, CTRL_ADDR, GEST_ID_ADDR, G_MODE_ADDR, I2C_ADDR, PERIOD_ACTIVE_ADDR,
        TIME_ENTER_MONITOR_ADDR,
    },
    ControlMode, DriverError, InterruptMode, TouchEvent,
};

/// An FT6x06 device.
///
/// This struct contains the full async interface to the FT6x06 device.
/// Certain features depend upon the specific FT6x06 chip and firmware version and some data or functionality may be unavailable.
pub struct FT6x06Async<I2C, P> {
    i2c: I2C,
    irq_pin: Option<P>,
}

impl<I2C: I2c<SevenBitAddress>, P: Wait> FT6x06Async<I2C, P> {
    pub fn new(i2c: I2C) -> Self {
        Self { i2c, irq_pin: None }
    }

    /// Set the IRQ pin.
    ///
    /// Use before calling [`FT6x06Async::wait_for_touch_event`].
    pub fn with_irq_pin(mut self, pin: P) -> Self {
        self.irq_pin = Some(pin);
        self
    }

    /// Wait for the next touch event.
    ///
    /// If the IRQ pin isn't set, this will return [`DriverError::IrqPinNotSet`].
    /// Set the IRQ pin with [`FT6x06Async::with_irq_pin`].
    pub async fn wait_for_touch(&mut self) -> Result<Option<TouchEvent>, DriverError<I2C::Error>> {
        let Some(ref mut irq_pin) = self.irq_pin else {
            return Err(DriverError::IrqPinNotSet);
        };

        irq_pin
            .wait_for_falling_edge()
            .await
            .map_err(|_| DriverError::IrqError)?;
        self.get_touch_event().await
    }

    /// Retrieve the latest touch event with all available data and touch points.
    pub async fn get_touch_event(&mut self) -> Result<Option<TouchEvent>, DriverError<I2C::Error>> {
        // Read from 0x01 to 0x0E for GEST_ID, TD_STATUS, and PX_X registers.
        let mut buf = [0u8; 14];
        self.i2c
            .write_read(I2C_ADDR, &[GEST_ID_ADDR], &mut buf)
            .await?;
        Ok(common::touch_event_from_buf(&buf))
    }

    /// Get the current mode of the device.
    /// This corresponds to the `CTRL` register.
    pub async fn get_control_mode(&mut self) -> Result<ControlMode, DriverError<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.i2c
            .write_read(I2C_ADDR, &[CTRL_ADDR], &mut buf)
            .await?;

        let value = ControlMode::try_from(buf[0]).map_err(|()| DriverError::InvalidResponse)?;
        Ok(value)
    }

    /// Set the device mode.
    /// The default control mode is [`ControlMode::MonitorIdle`].
    /// This corresponds to the `CTRL` register.
    pub async fn set_control_mode(
        &mut self,
        mode: ControlMode,
    ) -> Result<(), DriverError<I2C::Error>> {
        Ok(self.i2c.write(I2C_ADDR, &[CTRL_ADDR, mode as u8]).await?)
    }

    /// Get the idle timeout for the [`ControlMode::MonitorIdle`] mode.
    /// This corresponds to the `TIMEENTERMONITOR` register.
    pub async fn get_active_idle_timeout(&mut self) -> Result<u8, DriverError<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.i2c
            .write_read(I2C_ADDR, &[TIME_ENTER_MONITOR_ADDR], &mut buf)
            .await?;

        Ok(buf[0])
    }

    /// Set the idle timeout for the [`ControlMode::MonitorIdle`] mode.
    ///
    /// The default value is 10. The datasheet does not specify what type this value is (maybe seconds?).
    /// This corresponds to the `TIMEENTERMONITOR` register.
    pub async fn set_active_idle_timeout(
        &mut self,
        timeout: u8,
    ) -> Result<(), DriverError<I2C::Error>> {
        Ok(self
            .i2c
            .write(I2C_ADDR, &[TIME_ENTER_MONITOR_ADDR, timeout])
            .await?)
    }

    /// Get the configured report rates for each mode.
    /// This corresponds to the `PERIODACTIVE` and `PERIODMONITOR` registers.
    pub async fn get_report_rates(&mut self) -> Result<(u8, u8), DriverError<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(I2C_ADDR, &[PERIOD_ACTIVE_ADDR], &mut buf)
            .await?;
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
    pub async fn set_report_rates(
        &mut self,
        active_rate: u8,
        monitor_rate: u8,
    ) -> Result<(), DriverError<I2C::Error>> {
        Ok(self
            .i2c
            .write(I2C_ADDR, &[PERIOD_ACTIVE_ADDR, active_rate, monitor_rate])
            .await?)
    }

    /// Get the current interrupt mode.
    /// This corresponds to the `G_MODE` register.
    pub async fn get_interrupt_mode(&mut self) -> Result<InterruptMode, DriverError<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.i2c
            .write_read(I2C_ADDR, &[G_MODE_ADDR], &mut buf)
            .await?;

        let value = InterruptMode::try_from(buf[0]).map_err(|()| DriverError::InvalidResponse)?;
        Ok(value)
    }

    /// Set the current interrupt mode.
    /// This corresponds to the `G_MODE` register.
    pub async fn set_interrupt_mode(
        &mut self,
        mode: InterruptMode,
    ) -> Result<(), DriverError<I2C::Error>> {
        Ok(self.i2c.write(I2C_ADDR, &[G_MODE_ADDR, mode as u8]).await?)
    }

    /// Read a byte from any register.
    /// Provided as a catch-all for missing api.
    pub async fn read_register(&mut self, reg: u8) -> Result<u8, DriverError<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(I2C_ADDR, &[reg], &mut buf).await?;
        Ok(buf[0])
    }

    /// Write a byte to any register.
    /// Provided as a catch-all for missing api.
    ///
    /// # Safety
    /// This function allows you to write any value to any register and as such is marked unsafe.
    pub async unsafe fn write_register(
        &mut self,
        reg: u8,
        val: u8,
    ) -> Result<(), DriverError<I2C::Error>> {
        Ok(self.i2c.write(I2C_ADDR, &[reg, val]).await?)
    }

    /// Safely clean up the device, returning any owned peripherals.
    pub fn destroy(self) -> (I2C, Option<P>) {
        (self.i2c, self.irq_pin)
    }
}
