### ft6x06-rs
<div>
    <a href="https://crates.io/crates/ft6x06-rs">
        <img src="https://img.shields.io/crates/v/ft6x06-rs" alt="Crates.io version" />
    </a>
    <a href="https://docs.rs/ft6x06-rs">
        <img src="https://img.shields.io/badge/docs-latest-blue.svg" alt="docs.rs docs" />
    </a>
</div>
<br/>

`ft6x06-rs` is a pure-Rust `embedded-hal`-based driver for the I2C-based `ft6x06` capacitive touch screen controller. This crate aims to provide high-level functionality for most use-cases.

<hr/>
<br/>

**To-Do:**
- [x] `embedded-hal`-based driver.
- [x] `embedded-hal-async`-based driver.
- [ ] Figure out missing information for specific functionality.
- [ ] Add tests.


**Features**
- `sync-driver` - default, sync driver using `embedded-hal`.
- `async-driver` - async version of the driver using `embedded-hal-async`.

**Example**
```rs
// Initialization
let mut dev = FT6x06::new(i2c);

// Configure the device.
dev.set_interrupt_mode(InterruptMode::Trigger)?;
dev.set_control_mode(ControlMode::MonitorIdle)?;
dev.set_active_idle_timeout(10)?;
dev.set_report_rates(60, 25)?;

// Read the device configuration.
let interrupt_mode = dev.get_interrupt_mode()?;
let control_mode = dev.get_control_mode()?;
let active_idle_timeout = dev.get_active_idle_timeout()?;
let (active_rate, monitor_rate) = dev.get_report_rates()?;

info!("Irq Mode: {}", interrupt_mode);
info!("Ctrl Mode: {}", control_mode);
info!("Active Idle Timeout: {}", active_idle_timeout);
info!("Active Rate: {}", active_rate);
info!("Monitor Rate: {}", monitor_rate);

// Get the latest touch data. Usually after receiving an interrupt from the device.
let touch_event = dev.get_touch_event()?;

// In the async driver, you can use additionally wait for the next touch event:
let mut irq_pin = Input::new(my_periph.GPIO_XX, Pull::Up);
let mut dev_async = FT6x06Async::new(i2c);
 
loop {
    let touch_event = dev_async.wait_for_touch(&mut irq_pin).await?;
    defmt::info!("{:?}", touch_event);
}
```
