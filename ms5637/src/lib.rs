#![no_std]
#![doc = include_str!("../README.md")]

pub use ms56xx::{Error, Measurement, OversamplingExtended as Oversampling};

use ms56xx::{I2cInterface, Ms56xx, Ms5637 as Ms5637Variant};

// Typestate markers for sensor variants

/// MS5637 sensor driver.
///
/// Generic over the interface type (I2C only - MS5637 doesn't support SPI).
pub struct Ms5637<INTERFACE> {
    inner: Ms56xx<INTERFACE, Ms5637Variant>,
}

// Constructors for I2C
impl<I2C> Ms5637<I2cInterface<I2C>> {
    /// Create a new MS5637 sensor driver using I2C.
    ///
    /// The MS5637 has a fixed I2C address of 0x76.
    ///
    /// Note: You must call [`init`](Self::init) or [`init_blocking`](Self::init_blocking) before measurements.
    pub fn new_i2c(i2c: I2C) -> Self {
        Self {
            inner: Ms56xx::new_i2c(i2c, false), // csb_high doesn't matter for MS5637
        }
    }

    /// Get the configured I2C address of the sensor (always 0x76).
    pub fn address(&self) -> u8 {
        self.inner.address()
    }

    /// Release the underlying I2C bus.
    pub fn destroy(self) -> I2C {
        self.inner.destroy()
    }
}

// Common methods for all interfaces
impl<INTERFACE> Ms5637<INTERFACE> {
    /// Returns true if the sensor has been initialized successfully.
    pub fn is_initialized(&self) -> bool {
        self.inner.is_initialized()
    }
}

// Async methods for I2C
impl<I2C: embedded_hal_async::i2c::I2c> Ms5637<I2cInterface<I2C>> {
    /// Send reset command and wait for internal PROM reload (~3ms).
    ///
    /// # Errors
    /// Returns an error if interface communication fails.
    pub async fn reset(
        &mut self,
        delay: &mut impl embedded_hal_async::delay::DelayNs,
    ) -> Result<(), I2C::Error> {
        self.inner.reset(delay).await
    }

    /// Initialize the sensor by resetting it, reading PROM data and verifying the CRC.
    ///
    /// # Errors
    /// Returns [`Error::BusError`] if interface communication fails, or [`Error::CrcMismatch`] if the
    /// PROM CRC check fails.
    pub async fn init(
        &mut self,
        delay: &mut impl embedded_hal_async::delay::DelayNs,
    ) -> Result<(), Error<I2C::Error>> {
        self.inner.init(delay).await
    }

    /// Carry out a complete measurement and conversion of pressure and temperature.
    ///
    /// # Errors
    /// Returns [`Error::NotInitialized`] if the sensor has not been initialized yet, or
    /// [`Error::BusError`] if interface communication fails.
    pub async fn measure(
        &mut self,
        osr: Oversampling,
        delay: &mut impl embedded_hal_async::delay::DelayNs,
    ) -> Result<Measurement, Error<I2C::Error>> {
        self.inner.measure(osr, delay).await
    }
}

// Blocking methods for I2C
impl<I2C: embedded_hal::i2c::I2c> Ms5637<I2cInterface<I2C>> {
    /// Send reset command and wait for internal PROM reload (~3ms).
    ///
    /// This is the blocking version.
    ///
    /// # Errors
    /// Returns an error if interface communication fails.
    pub fn reset_blocking(
        &mut self,
        delay: &mut impl embedded_hal::delay::DelayNs,
    ) -> Result<(), I2C::Error> {
        self.inner.reset_blocking(delay)
    }

    /// Initialize the sensor by resetting it, reading PROM data and verifying the CRC.
    ///
    /// This is the blocking version.
    ///
    /// # Errors
    /// Returns [`Error::BusError`] if interface communication fails, or [`Error::CrcMismatch`] if the
    /// PROM CRC check fails.
    pub fn init_blocking(
        &mut self,
        delay: &mut impl embedded_hal::delay::DelayNs,
    ) -> Result<(), Error<I2C::Error>> {
        self.inner.init_blocking(delay)
    }

    /// Carry out a complete measurement and conversion of pressure and temperature.
    ///
    /// This is the blocking version.
    ///
    /// # Errors
    /// Returns [`Error::NotInitialized`] if the sensor has not been initialized yet, or
    /// [`Error::BusError`] if interface communication fails.
    pub fn measure_blocking(
        &mut self,
        osr: Oversampling,
        delay: &mut impl embedded_hal::delay::DelayNs,
    ) -> Result<Measurement, Error<I2C::Error>> {
        self.inner.measure_blocking(osr, delay)
    }
}
