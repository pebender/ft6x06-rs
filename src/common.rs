//! Common helper functions and constants.
//!
//! This module contains common data processing functions used by both sync and async drivers.

use crate::{GestureType, TouchEvent, TouchPoint, TouchType};

// Register addresses
pub(crate) const I2C_ADDR: u8 = 0x38;
pub(crate) const GEST_ID_ADDR: u8 = 0x01;
pub(crate) const CTRL_ADDR: u8 = 0x86;
pub(crate) const TIME_ENTER_MONITOR_ADDR: u8 = 0x87;
pub(crate) const PERIOD_ACTIVE_ADDR: u8 = 0x88;
pub(crate) const G_MODE_ADDR: u8 = 0xA4;

/// Extract a full touch event from a batch i2c read.
///
/// `GEST_ID_ADDR` -> `GEST_ID_ADDR` + 14
pub(crate) fn touch_event_from_buf(buf: &[u8; 14]) -> Option<TouchEvent> {
    // Get the number of touch points.
    // Less than 1 = no data; Greater than 2 = invalid.
    let num_touch_points = number_touch_points_from_register(buf[1]);
    if !is_num_touch_points_valid(num_touch_points) {
        return None;
    }

    let gesture = GestureType::from(buf[0]);

    // Get first touch point
    let primary_point = touch_point_from_registers(buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);

    // Get second touch point
    let mut secondary_point = None;
    if num_touch_points == 2 {
        let touch_point =
            touch_point_from_registers(buf[8], buf[9], buf[10], buf[11], buf[12], buf[13]);
        secondary_point = Some(touch_point);
    }

    Some(TouchEvent {
        primary_point,
        secondary_point,
        gesture,
    })
}

/*
 -- Common register data processing --
*/

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
