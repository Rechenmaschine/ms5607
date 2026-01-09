#![no_std]
#![doc = include_str!("../README.md")]

pub use ms56xx::{Error, Measurement, OversamplingStandard as Oversampling};

use ms56xx::{I2cInterface, Ms56xx, Ms5611 as Ms5611Variant, SpiInterface};

/// MS5611 sensor driver.
///
/// Generic over the interface type (I2C or SPI).
pub struct Ms5611<INTERFACE> {
    inner: Ms56xx<INTERFACE, Ms5611Variant>,
}

// Constructors for I2C
impl<I2C> Ms5611<I2cInterface<I2C>> {
    /// Create a new MS5611 sensor driver using I2C with address based on CSB pin state.
    ///   - `csb_high` = true  => 0x76
    ///   - `csb_high` = false => 0x77
    ///
    /// Note: You must call [`init`](Self::init) or [`init_blocking`](Self::init_blocking) before measurements.
    pub fn new_i2c(i2c: I2C, csb_high: bool) -> Self {
        Self {
            inner: Ms56xx::new_i2c(i2c, csb_high),
        }
    }

    /// Get the configured I2C address of the sensor.
    pub fn address(&self) -> u8 {
        self.inner.address()
    }

    /// Release the underlying I2C bus.
    pub fn destroy(self) -> I2C {
        self.inner.destroy()
    }
}

// Constructors for SPI
impl<SPI> Ms5611<SpiInterface<SPI>> {
    /// Create a new MS5611 sensor driver using SPI.
    ///
    /// Note: CS (chip select) must be handled by the provided `SpiDevice`.
    /// You must call [`init`](Self::init) or [`init_blocking`](Self::init_blocking) before measurements.
    pub fn new_spi(spi: SPI) -> Self {
        Self {
            inner: Ms56xx::new_spi(spi),
        }
    }

    /// Release the underlying SPI device.
    pub fn destroy(self) -> SPI {
        self.inner.destroy()
    }
}

// Common methods for all interfaces
impl<INTERFACE> Ms5611<INTERFACE> {
    /// Returns true if the sensor has been initialized successfully.
    pub fn is_initialized(&self) -> bool {
        self.inner.is_initialized()
    }
}

// Async methods for I2C
impl<I2C: embedded_hal_async::i2c::I2c> Ms5611<I2cInterface<I2C>> {
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
impl<I2C: embedded_hal::i2c::I2c> Ms5611<I2cInterface<I2C>> {
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

// Async methods for SPI
impl<SPI: embedded_hal_async::spi::SpiDevice> Ms5611<SpiInterface<SPI>> {
    /// Send reset command and wait for internal PROM reload (~3ms).
    ///
    /// # Errors
    /// Returns an error if interface communication fails.
    pub async fn reset(
        &mut self,
        delay: &mut impl embedded_hal_async::delay::DelayNs,
    ) -> Result<(), SPI::Error> {
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
    ) -> Result<(), Error<SPI::Error>> {
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
    ) -> Result<Measurement, Error<SPI::Error>> {
        self.inner.measure(osr, delay).await
    }
}

// Blocking methods for SPI
impl<SPI: embedded_hal::spi::SpiDevice> Ms5611<SpiInterface<SPI>> {
    /// Send reset command and wait for internal PROM reload (~3ms).
    ///
    /// This is the blocking version.
    ///
    /// # Errors
    /// Returns an error if interface communication fails.
    pub fn reset_blocking(
        &mut self,
        delay: &mut impl embedded_hal::delay::DelayNs,
    ) -> Result<(), SPI::Error> {
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
    ) -> Result<(), Error<SPI::Error>> {
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
    ) -> Result<Measurement, Error<SPI::Error>> {
        self.inner.measure_blocking(osr, delay)
    }
}
