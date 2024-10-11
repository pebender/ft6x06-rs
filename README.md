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
- [ ] `embedded-hal-async`-based driver.
- [ ] Figure out missing information for specific functionality.
- [ ] Add testing.


### Example
```rs
// Initialization
let mut dev = FT6x06::new(i2c);

// Configure the device.
dev.set_interrupt_mode(InterruptMode::Trigger).unwrap();
dev.set_control_mode(ControlMode::MonitorIdle).unwrap();
dev.set_active_idle_timeout(10).unwrap();
dev.set_report_rates(60, 25).unwrap();

// Read the device configuration.
let interrupt_mode = dev.get_interrupt_mode().unwrap();
let control_mode = dev.get_control_mode().unwrap();
let active_idle_timeout = dev.get_active_idle_timeout().unwrap();
let (active_rate, monitor_rate) = dev.get_report_rates().unwrap();

info!("Irq Mode: {}", interrupt_mode);
info!("Ctrl Mode: {}", control_mode);
info!("Active Idle Timeout: {}", active_idle_timeout);
info!("Active Rate: {}", active_rate);
info!("Monitor Rate: {}", monitor_rate);

// Get the latest touch data. Usually after receiving an interrupt from the device.
let touch_event = dev.get_touch_event().unwrap();
```
