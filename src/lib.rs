#![no_std]
#![doc = include_str!("../README.md")]

use embedded_hal::{delay::DelayNs as BlockingDelay, i2c::I2c as BlockingI2c};
use embedded_hal_async::{delay::DelayNs as AsyncDelay, i2c::I2c as AsyncI2c};

/// A driver for the MS5607 pressure sensor.
pub struct Ms5607<I2C> {
    i2c: I2C,
    addr: u8,
    prom: [u16; 8], // C0..C7 (7 calibration words + CRC)
    initialized: bool,
}

/// Which conversion to request from the ADC.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
enum ConversionType {
    /// Pressure conversion (D1).
    D1,
    /// Temperature conversion (D2).
    D2,
}

/// The compensated measurement result.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Measurement {
    /// Pressure in millibar (hPa)
    pub pressure_mbar: f32,
    /// Temperature in °C
    pub temperature_c: f32,
}

/// Oversampling ratio. Higher => more precise but slower.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum Oversampling {
    Osr256 = 0x00,  // ~0.6ms
    Osr512 = 0x02,  // ~1.1ms
    Osr1024 = 0x04, // ~2.1ms
    Osr2048 = 0x06, // ~4.2ms
    Osr4096 = 0x08, // ~8.5ms
}

/// Error type for MS5607 operations.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum Error<E> {
    /// I2C bus error
    BusError(E),
    /// CRC mismatch in PROM
    CrcMismatch,
    /// Sensor not initialized (call [`Ms5607::init`] first)
    NotInitialized,
}

// I2C commands from datasheet.
const CMD_RESET: u8 = 0x1E; // reset
const CMD_ADC_READ: u8 = 0x00; // read ADC (24-bit)
const CMD_CONVERT_D1: u8 = 0x40; // start pressure conversion
const CMD_CONVERT_D2: u8 = 0x50; // start temperature conversion
const CMD_PROM_BASE: u8 = 0xA0; // read PROM base (then +2 * index)

// ============================================================================
// Trait-agnostic methods (shared between async and blocking)
// ============================================================================

impl<I2C> Ms5607<I2C> {
    /// Create a new sensor driver with the address
    ///   - `csb_high` = true  => 0x76
    ///   - `csb_high` = false => 0x77
    ///
    /// Note that this does not initialize the sensor; you must call [`Ms5607::init`] before
    /// performing measurements.
    pub fn new(i2c: I2C, csb_high: bool) -> Self {
        Self::new_with_addr(i2c, if csb_high { 0x76 } else { 0x77 })
    }

    /// Create a new sensor driver with an address. The address depends on the CSB pin state.
    /// Note that this does not initialize the sensor; you must call [`Ms5607::init`] before
    /// performing measurements.
    pub fn new_with_addr(i2c: I2C, addr: u8) -> Self {
        Self {
            i2c,
            addr,
            prom: [0; 8],
            initialized: false,
        }
    }

    /// Release the I2C bus
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    /// Get the configured I2C address of the sensor
    pub fn address(&self) -> u8 {
        self.addr
    }

    /// Returns true if the sensor has been initialized successfully.
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Check the CRC4 of the PROM. Returns true if it matches the stored value.
    #[allow(arithmetic_overflow)]
    fn check_crc4(&self) -> bool {
        let stored_crc = (self.prom[7] & 0x000F) as u8;
        let computed_crc = calculate_crc4(&self.prom);
        computed_crc == stored_crc
    }

    /// Convert raw D1 & D2 into compensated values using formula from datasheet
    /// formula (incl. 2nd order).
    #[allow(clippy::cast_possible_truncation)]
    fn compensate(&self, d1: u32, d2: u32) -> (i32, i32) {
        // PROM layout
        //   prom[1] = C1 (Pressure Sens)
        //   prom[2] = C2 (Pressure Offset)
        //   prom[3] = C3 (Temp Coeff of Press Sens)
        //   prom[4] = C4 (Temp Coeff of Press Offset)
        //   prom[5] = C5 (Ref Temp)
        //   prom[6] = C6 (Temp Coeff of Temp)
        let c1 = i64::from(self.prom[1]);
        let c2 = i64::from(self.prom[2]);
        let c3 = i64::from(self.prom[3]);
        let c4 = i64::from(self.prom[4]);
        let c5 = i64::from(self.prom[5]);
        let c6 = i64::from(self.prom[6]);

        // Convert to i64
        let d1 = i64::from(d1);
        let d2 = i64::from(d2);

        // First-order
        let dt = d2 - (c5 << 8);
        let temp = 2000 + (dt * c6) / (1 << 23);
        let off = (c2 << 17) + ((c4 * dt) >> 6);
        let sens = (c1 << 16) + ((c3 * dt) >> 7);

        // Second-order
        let mut t2 = 0i64;
        let mut off2 = 0i64;
        let mut sens2 = 0i64;

        if temp < 2000 {
            let t_low = temp - 2000; // (temp in hundredths of °C)
            t2 = (dt * dt) >> 31;
            off2 = (61 * t_low * t_low) >> 4;
            sens2 = 2 * t_low * t_low;
            // Additional correction if below -15°C (temp < -1500)
            if temp < -1500 {
                let temp_very_low = temp + 1500;
                off2 += 15 * temp_very_low * temp_very_low;
                sens2 += 8 * temp_very_low * temp_very_low;
            }
        }

        // Apply 2nd order
        let temp_corrected = temp - t2;
        let off_corrected = off - off2;
        let sens_corrected = sens - sens2;

        let p = (((d1 * sens_corrected) >> 21) - off_corrected) >> 15;
        (p as i32, temp_corrected as i32)
    }
}

// ============================================================================
// Async API
// ============================================================================

impl<I2C> Ms5607<I2C>
where
    I2C: AsyncI2c,
{
    /// Send reset command, and wait ~3 ms for internal PROM reload.
    ///
    /// # Errors
    /// Returns an error if the I2C write fails.
    pub async fn reset(&mut self, delay: &mut impl AsyncDelay) -> Result<(), I2C::Error> {
        self.i2c.write(self.addr, &[CMD_RESET]).await?;
        delay.delay_ms(3).await; // ~2.8 ms in datasheet
        Ok(())
    }

    /// Read all 8 words from PROM (each 16 bits, big-endian).
    /// The 0th word is factory data, 1..6 are calibration, 7 is CRC.
    async fn read_prom_all(&mut self) -> Result<(), I2C::Error> {
        for i in 0..8u8 {
            let cmd = CMD_PROM_BASE + (i * 2);
            let mut buf = [0u8; 2];
            self.i2c.write_read(self.addr, &[cmd], &mut buf).await?;

            // Coefficients are big-endian, so we decode accordingly
            self.prom[i as usize] = u16::from_be_bytes(buf);
        }
        Ok(())
    }

    /// Initialize the sensor by resetting it, reading PROM data and verifying the CRC.
    ///
    /// # Errors
    /// Returns [`Error::BusError`] if I2C communication fails, or [`Error::CrcMismatch`] if the
    /// PROM CRC check fails.
    pub async fn init(&mut self, delay: &mut impl AsyncDelay) -> Result<(), Error<I2C::Error>> {
        self.reset(delay).await.map_err(Error::BusError)?;
        self.read_prom_all().await.map_err(Error::BusError)?;

        if !self.check_crc4() {
            return Err(Error::CrcMismatch);
        }

        self.initialized = true;
        Ok(())
    }

    /// Start a conversion: D1 (pressure) or D2 (temperature) with selected oversampling.
    async fn start_conversion(
        &mut self,
        conv: ConversionType,
        osr: Oversampling,
    ) -> Result<(), I2C::Error> {
        let cmd = match conv {
            ConversionType::D1 => CMD_CONVERT_D1,
            ConversionType::D2 => CMD_CONVERT_D2,
        } | (osr as u8);
        self.i2c.write(self.addr, &[cmd]).await?;
        Ok(())
    }

    /// Start a conversion, wait, then read the ADC result.
    ///
    /// The ADC result is a 24-bit value in big-endian format.
    async fn read_adc(&mut self) -> Result<u32, I2C::Error> {
        // We read 3 data bytes into indices [1..4] of buf
        let mut buf = [0u8; 4];
        self.i2c
            .write_read(self.addr, &[CMD_ADC_READ], &mut buf[1..4])
            .await?;

        // buf = [0, hi, mid, lo]
        let val = u32::from_be_bytes(buf);
        Ok(val)
    }

    /// Waits for the conversion to complete based on the oversampling ratio and the
    /// max conversion times from the datasheet.
    async fn wait_conversion(&mut self, osr: Oversampling, delay: &mut impl AsyncDelay) {
        // max conversion time in ms from datasheet
        let us = match osr {
            Oversampling::Osr256 => 600,   // 0.60 ms
            Oversampling::Osr512 => 1170,  // 1.17 ms
            Oversampling::Osr1024 => 2280, // 2.28 ms
            Oversampling::Osr2048 => 4540, // 4.54 ms
            Oversampling::Osr4096 => 9040, // 9.04 ms
        };
        delay.delay_us(us).await;
    }

    /// Carry out a complete measurement and conversion of pressure and temperature, and
    /// compensate the raw ADC values using the PROM coefficients.
    ///
    /// # Errors
    /// Returns [`Error::NotInitialized`] if the sensor has not been initialized yet, or
    /// [`Error::BusError`] if I2C communication fails.
    pub async fn measure(
        &mut self,
        osr: Oversampling,
        delay: &mut impl AsyncDelay,
    ) -> Result<Measurement, Error<I2C::Error>> {
        if !self.initialized {
            return Err(Error::NotInitialized);
        }

        // 1) Start pressure readout (D1), wait, read
        self.start_conversion(ConversionType::D1, osr)
            .await
            .map_err(Error::BusError)?;
        self.wait_conversion(osr, delay).await;
        let d1_raw = self.read_adc().await.map_err(Error::BusError)?;

        // 2) Start temperature readout (D2), wait, read
        self.start_conversion(ConversionType::D2, osr)
            .await
            .map_err(Error::BusError)?;
        self.wait_conversion(osr, delay).await;
        let d2_raw = self.read_adc().await.map_err(Error::BusError)?;

        // 3) Compensate raw values
        let (p_i, t_i) = self.compensate(d1_raw, d2_raw);

        #[allow(clippy::cast_precision_loss)]
        Ok(Measurement {
            pressure_mbar: p_i as f32 / 100.0,
            temperature_c: t_i as f32 / 100.0,
        })
    }
}

// ============================================================================
// Blocking API
// ============================================================================

impl<I2C> Ms5607<I2C>
where
    I2C: BlockingI2c,
{
    /// Send reset command, and wait ~3 ms for internal PROM reload.
    ///
    /// This is the blocking version of [`Ms5607::reset`].
    ///
    /// # Errors
    /// Returns an error if the I2C write fails.
    pub fn reset_blocking(&mut self, delay: &mut impl BlockingDelay) -> Result<(), I2C::Error> {
        self.i2c.write(self.addr, &[CMD_RESET])?;
        delay.delay_ms(3); // ~2.8 ms in datasheet
        Ok(())
    }

    /// Read all 8 words from PROM (each 16 bits, big-endian).
    /// The 0th word is factory data, 1..6 are calibration, 7 is CRC.
    fn read_prom_all_blocking(&mut self) -> Result<(), I2C::Error> {
        for i in 0..8u8 {
            let cmd = CMD_PROM_BASE + (i * 2);
            let mut buf = [0u8; 2];
            self.i2c.write_read(self.addr, &[cmd], &mut buf)?;

            // Coefficients are big-endian, so we decode accordingly
            self.prom[i as usize] = u16::from_be_bytes(buf);
        }
        Ok(())
    }

    /// Initialize the sensor by resetting it, reading PROM data and verifying the CRC.
    ///
    /// This is the blocking version of [`Ms5607::init`].
    ///
    /// # Errors
    /// Returns [`Error::BusError`] if I2C communication fails, or [`Error::CrcMismatch`] if the
    /// PROM CRC check fails.
    pub fn init_blocking(
        &mut self,
        delay: &mut impl BlockingDelay,
    ) -> Result<(), Error<I2C::Error>> {
        self.reset_blocking(delay).map_err(Error::BusError)?;
        self.read_prom_all_blocking().map_err(Error::BusError)?;

        if !self.check_crc4() {
            return Err(Error::CrcMismatch);
        }

        self.initialized = true;
        Ok(())
    }

    /// Start a conversion: D1 (pressure) or D2 (temperature) with selected oversampling.
    fn start_conversion_blocking(
        &mut self,
        conv: ConversionType,
        osr: Oversampling,
    ) -> Result<(), I2C::Error> {
        let cmd = match conv {
            ConversionType::D1 => CMD_CONVERT_D1,
            ConversionType::D2 => CMD_CONVERT_D2,
        } | (osr as u8);
        self.i2c.write(self.addr, &[cmd])?;
        Ok(())
    }

    /// Read the ADC result.
    ///
    /// The ADC result is a 24-bit value in big-endian format.
    fn read_adc_blocking(&mut self) -> Result<u32, I2C::Error> {
        // We read 3 data bytes into indices [1..4] of buf
        let mut buf = [0u8; 4];
        self.i2c
            .write_read(self.addr, &[CMD_ADC_READ], &mut buf[1..4])?;

        // buf = [0, hi, mid, lo]
        let val = u32::from_be_bytes(buf);
        Ok(val)
    }

    /// Waits for the conversion to complete based on the oversampling ratio and the
    /// max conversion times from the datasheet.
    fn wait_conversion_blocking(osr: Oversampling, delay: &mut impl BlockingDelay) {
        // max conversion time in us from datasheet
        let us = match osr {
            Oversampling::Osr256 => 600,   // 0.60 ms
            Oversampling::Osr512 => 1170,  // 1.17 ms
            Oversampling::Osr1024 => 2280, // 2.28 ms
            Oversampling::Osr2048 => 4540, // 4.54 ms
            Oversampling::Osr4096 => 9040, // 9.04 ms
        };
        delay.delay_us(us);
    }

    /// Carry out a complete measurement and conversion of pressure and temperature, and
    /// compensate the raw ADC values using the PROM coefficients.
    ///
    /// # Errors
    /// Returns [`Error::NotInitialized`] if the sensor has not been initialized yet, or
    /// [`Error::BusError`] if I2C communication fails.
    ///
    /// This is the blocking version of [`Ms5607::measure`].
    pub fn measure_blocking(
        &mut self,
        osr: Oversampling,
        delay: &mut impl BlockingDelay,
    ) -> Result<Measurement, Error<I2C::Error>> {
        if !self.initialized {
            return Err(Error::NotInitialized);
        }

        // 1) Start pressure readout (D1), wait, read
        self.start_conversion_blocking(ConversionType::D1, osr)
            .map_err(Error::BusError)?;
        Self::wait_conversion_blocking(osr, delay);
        let d1_raw = self.read_adc_blocking().map_err(Error::BusError)?;

        // 2) Start temperature readout (D2), wait, read
        self.start_conversion_blocking(ConversionType::D2, osr)
            .map_err(Error::BusError)?;
        Self::wait_conversion_blocking(osr, delay);
        let d2_raw = self.read_adc_blocking().map_err(Error::BusError)?;

        // 3) Compensate raw values
        let (p_i, t_i) = self.compensate(d1_raw, d2_raw);

        #[allow(clippy::cast_precision_loss)]
        Ok(Measurement {
            pressure_mbar: p_i as f32 / 100.0,
            temperature_c: t_i as f32 / 100.0,
        })
    }
}

/// Computes the CRC4 checksum for the PROM coefficients, as specified
/// in the AN520 datasheet.
#[allow(arithmetic_overflow)]
fn calculate_crc4(n_prom: &[u16; 8]) -> u8 {
    let mut copy = *n_prom;
    copy[7] &= 0xFF00;
    let mut n_rem: u16 = 0;
    for cnt in 0..16 {
        let byte = if cnt % 2 == 0 {
            copy[cnt >> 1] >> 8
        } else {
            copy[cnt >> 1] & 0x00FF
        };
        n_rem ^= byte;
        for _ in 0..8 {
            if (n_rem & 0x8000) != 0 {
                n_rem = (n_rem << 1) ^ 0x3000;
            } else {
                n_rem <<= 1;
            }
        }
    }
    // final 4-bit remainder is the CRC code
    ((n_rem >> 12) & 0x000F) as u8
}

#[cfg(test)]
mod tests {
    use crate::calculate_crc4;

    #[test]
    fn test_crc4() {
        // example PROM from the datasheet
        let n_prom: [u16; 8] = [
            0x3132, 0x3334, 0x3536, 0x3738, 0x3940, 0x4142, 0x4344, 0x4500,
        ];
        let crc = calculate_crc4(&n_prom);
        if crc != 0xB {
            panic!("CRC mismatch: expected 0xB, got 0x{:X}", crc);
        }

        let n_prom: [u16; 8] = [
            0x3132, 0x3334, 0x3536, 0x3738, 0x3940, 0x4142, 0x4344, 0x450B,
        ];
        let crc = calculate_crc4(&n_prom);

        let stored_crc = (n_prom[7] & 0x000F) as u8;
        if crc != stored_crc {
            panic!("CRC mismatch: expected 0xB, got 0x{:X}", crc);
        }
    }
}
