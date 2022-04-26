//! An embedded-hal driver for the Maxim MAX17205 fuel gauge.
//!
//! Loosely based on
//! https://github.com/tock/tock/blob/master/capsules/src/max17205.rs
//! rewritten to use the embedded-hal I2C driver.
//!
//! <https://www.maximintegrated.com/en/products/power/battery-management/MAX17205.html>
//!
//! > The MAX1720x/MAX1721x are ultra-low power stand-alone fuel gauge ICs that
//! > implement the Maxim ModelGauge™ m5 algorithm without requiring host
//! > interaction for configuration. This feature makes the MAX1720x/MAX1721x
//! > excellent pack-side fuel gauges. The MAX17201/MAX17211 monitor a single
//! > cell pack. The MAX17205/MAX17215 monitor and balance a 2S or 3S pack or
//! > monitor a multiple-series cell pack.
//!
//! Usage
//! -----
//!
//! use rppal::i2c::I2c;
//! fn main() {
//!     let mut i2c = I2c::new().unwrap();
//!     let mut max17205 = MAX1720x::new(&mut i2c);
//!     let soc = max17205.state_of_charge(&mut i2c).unwrap();
//!     let status = max17205.status(&mut i2c).unwrap();
//!     let voltage = max17205.voltage(&mut i2c).unwrap();
//!     let current = max17205.current(&mut i2c).unwrap();
//!     println!("State of charge: {}%", soc);
//!     println!("Voltage: {}V", voltage);
//!     println!("Current: {}A", current);
//!     println!("Status: {:#?}", status);
//! }

#![no_std]

use embedded_hal as hal;
use hal::blocking::i2c::{Read, Write, WriteRead};
use core::marker::PhantomData;

// Addresses 0x000 - 0x0FF, 0x180 - 0x1FF can be written as blocks
// Addresses 0x100 - 0x17F must be written by word

// Addresses 0x000 - 0x0FF should use the ADDR_LOWER address
// Addresses 0x100 - 0x1FF should use the ADDR_UPPER address

// Note that the datasheet gives addresses in 0-bit format with the RW bit
// set to 0.  We want the addresses in 7-bit format, i.e. the datasheet ones
// shifted right by one bit.
const ADDR_LOWER: u8 = 0x36;
const ADDR_UPPER: u8 = 0x0b;

#[allow(dead_code)]
#[repr(u16)]
enum Registers {
    Status = 0x000,     // Status flags
    RepCap = 0x005,     // Reported capacity, LSB = 0.5 mAh
    RepSOC = 0x006,     // Reported capacity, LSB = %/256
    Voltage = 0x009,    // The lowest reading from all cell voltages, LSB = 0.078125 mV
    Current = 0x00A,    // Instantaneous current, LSB = 156.25 uA
    Tte = 0x011,        // Time To Empty
    Ttf = 0x020,        // Time to Full
    FullCapRep = 0x035, // Maximum capacity, LSB = 0.5 mAh
    Coulomb = 0x04D,    // Raw coloumb count
    Batt = 0x0DA,       // Pack voltage, LSB = 1.25mV
    NPackCfg = 0x1B5,   // Pack configuration
    NRomID = 0x1BC,     // RomID - 64bit unique
    NRSense = 0x1CF,    // Sense resistor
}

/// Return the I2C device address used to communicate when accessing this
/// register
fn device_addr(reg: Registers) -> u8 {
    if reg as u16 > 0x100 {
        ADDR_UPPER
    } else {
        ADDR_LOWER
    }
}

/// Return the register address used to access this register
fn reg_addr(reg: Registers) -> u8 {
    ((reg as u16) & 0xff) as u8
}

#[allow(dead_code)]
#[derive(Debug)]
/// Represents the status of the MAX1720x fuel gauge IC read from the STATUS register
pub struct Status {
    /// Power-On Reset
    por: bool,
    /// Minimum current alert threshold exceeded
    imn: bool,
    /// Battery status
    bst: bool,
    /// Maximum currentl alert threshold exceeded
    imx: bool,
    /// State of charge 1% change alert
    dsoci: bool,
    /// Minimum voltage alert threshold exceeded
    vmn: bool,
    /// Minimum temperature alert threshold exceeded
    tmn: bool,
    /// Minimum SOC alert threshold exceeded
    smn: bool,
    /// Battery insertion
    bi: bool,
    /// Maximum voltage alert threshold exceeded
    vmx: bool,
    /// Maximum temperature alert threshold exceeded
    tmx: bool,
    /// Maximum SOC alert threshold exceeded
    smx: bool,
    /// Battery removal
    br: bool,
}

pub struct MAX1720x<I2C, E> {
    phantom: PhantomData<I2C>,
    phantom_e: PhantomData<E>,
}

impl<I2C, E> MAX1720x<I2C, E>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
{
    /// Make a new MAX17205 driver
    pub fn new(_bus: &mut I2C) -> Self {
        Self {
            phantom: PhantomData,
            phantom_e: PhantomData,
        }
    }

    /// Get the fuel gauge status
    pub fn status(&mut self, bus: &mut I2C) -> Result<Status, E> {
        let mut raw = [0u8; 2];
        let dev_addr = device_addr(Registers::Status);
        let reg_addr = reg_addr(Registers::Status);
        bus.write_read(dev_addr, &[reg_addr], &mut raw)?;
        let raw = ((raw[1] as u16) << 8) | (raw[0] as u16);
        Ok(Status {
            br: raw & (1 << 15) != 0,
            smx: raw & (1 << 14) != 0,
            tmx: raw & (1 << 13) != 0,
            vmx: raw & (1 << 12) != 0,
            bi: raw & (1 << 11) != 0,
            smn: raw & (1 << 10) != 0,
            tmn: raw & (1 << 9) != 0,
            vmn: raw & (1 << 8) != 0,
            dsoci: raw & (1 << 7) != 0,
            imx: raw & (1 << 6) != 0,
            bst: raw & (1 << 3) != 0,
            imn: raw & (1 << 2) != 0,
            por: raw & (1 << 1) != 0,
        })
    }

    /// Get the current estimated state of charge as a percentage
    pub fn state_of_charge(&mut self, bus: &mut I2C) -> Result<f32, E> {
        let mut raw = [0u8; 2];
        let dev_addr = device_addr(Registers::RepSOC);
        let reg_addr = reg_addr(Registers::RepSOC);
        bus.write_read(dev_addr, &[reg_addr], &mut raw)?;
        let raw = ((raw[1] as u16) << 8) | (raw[0] as u16);
        // Conversion ratio from datasheet Table 1
        Ok((raw as f32) / 256.0)
    }

    /// Get the current pack voltage in volts
    pub fn voltage(&mut self, bus: &mut I2C) -> Result<f32, E> {
        let mut raw = [0u8; 2];
        let dev_addr = device_addr(Registers::Batt);
        let reg_addr = reg_addr(Registers::Batt);
        bus.write_read(dev_addr, &[reg_addr], &mut raw)?;
        let raw = ((raw[1] as u16) << 8) | (raw[0] as u16);
        // Conversion ratio from datasheet "Batt Register" register info
        Ok((raw as f32) * 0.001_25)
    }

    /// Get the current pack current in amps
    pub fn current(&mut self, bus: &mut I2C) -> Result<f32, E> {
        let mut raw = [0u8; 2];
        let dev_addr = device_addr(Registers::Current);
        let reg_addr = reg_addr(Registers::Current);
        bus.write_read(dev_addr, &[reg_addr], &mut raw)?;
        let raw = ((raw[1] as u16) << 8) | (raw[0] as u16);
        // Convert from twos complement form into a real signed integer
        let raw = raw as i16;
        // Conversion ratio from datasheet Table 1
        Ok((raw as f32) * 0.000_156_25)
    }
}
