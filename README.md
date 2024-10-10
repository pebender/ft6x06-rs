### ft6x06-rs
`ft6x06-rs` is a pure-Rust `embedded-hal`-based driver for the `ft6x06` capacitive touch screen controller. This crate aims to provide high-level functionality for most use-cases.

**To-Do:**
- [x] `embedded-hal`-based driver.
- [ ] `emedded-hal-async`-based driver.
- [ ] Ability to configure device.
- [ ] Add testing.


### Example
```rs
// Initialization
let mut dev = FT6x06::new(i2c);

// Get the latest touch data. Usually after receiving an interrupt from the device.
let touch_event = dev.get_touch_event().unwrap();
```
