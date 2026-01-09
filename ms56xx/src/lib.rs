#![no_std]
#![doc = include_str!("../README.md")]

use core::marker::PhantomData;

use embedded_hal::{
    delay::DelayNs as BlockingDelay, i2c::I2c as BlockingI2c, spi::SpiDevice as BlockingSpiDevice,
};
use embedded_hal_async::{
    delay::DelayNs as AsyncDelay, i2c::I2c as AsyncI2c, spi::SpiDevice as AsyncSpiDevice,
};

/// Error type for `MS56xx` operations.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum Error<E> {
    /// Bus error (I2C or SPI)
    BusError(E),
    /// CRC mismatch in PROM
    CrcMismatch,
    /// Sensor not initialized (call init first)
    NotInitialized,
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

/// Trait for oversampling ratio types.
pub trait OversamplingType: Copy {
    /// Get the command bits for this oversampling ratio.
    fn cmd_value(self) -> u8;

    /// Get the conversion delay in microseconds.
    fn delay_us(self) -> u32;
}

/// Oversampling ratios for MS5607 and MS5611 (256-4096).
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum OversamplingStandard {
    Osr256,
    Osr512,
    Osr1024,
    Osr2048,
    Osr4096,
}

impl OversamplingType for OversamplingStandard {
    fn cmd_value(self) -> u8 {
        match self {
            Self::Osr256 => 0x00,
            Self::Osr512 => 0x02,
            Self::Osr1024 => 0x04,
            Self::Osr2048 => 0x06,
            Self::Osr4096 => 0x08,
        }
    }

    fn delay_us(self) -> u32 {
        // Conservative delays (µs). If you want strict typ/max, model-specific OSR types are needed.
        match self {
            Self::Osr256 => 600,
            Self::Osr512 => 1170,
            Self::Osr1024 => 2280,
            Self::Osr2048 => 4540,
            Self::Osr4096 => 9040,
        }
    }
}

/// Oversampling ratios for MS5637 (256-8192).
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum OversamplingExtended {
    Osr256,
    Osr512,
    Osr1024,
    Osr2048,
    Osr4096,
    Osr8192,
}

impl OversamplingType for OversamplingExtended {
    fn cmd_value(self) -> u8 {
        match self {
            Self::Osr256 => 0x00,
            Self::Osr512 => 0x02,
            Self::Osr1024 => 0x04,
            Self::Osr2048 => 0x06,
            Self::Osr4096 => 0x08,
            Self::Osr8192 => 0x0A,
        }
    }

    fn delay_us(self) -> u32 {
        // MS5637 typ. conversion times (ms): 0.54, 1.06, 2.08, 4.13, 8.22, 16.44
        match self {
            Self::Osr256 => 540,
            Self::Osr512 => 1060,
            Self::Osr1024 => 2080,
            Self::Osr2048 => 4130,
            Self::Osr4096 => 8220,
            Self::Osr8192 => 16440,
        }
    }
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

const CMD_RESET: u8 = 0x1E;
const CMD_ADC_READ: u8 = 0x00;
const CMD_CONVERT_D1: u8 = 0x40;
const CMD_CONVERT_D2: u8 = 0x50;
const CMD_PROM_BASE: u8 = 0xA0;

const RESET_DELAY_MS: u32 = 3;

/// Blocking interface operations for `MS56xx` sensors.
pub trait Ms56xxInterface {
    type Error;

    /// Write a command byte.
    ///
    /// # Errors
    /// Returns an error if the interface communication fails.
    fn write_cmd(&mut self, cmd: u8) -> Result<(), Self::Error>;

    /// Write a command byte and read the response.
    ///
    /// # Errors
    /// Returns an error if the interface communication fails.
    fn read(&mut self, cmd: u8, buf: &mut [u8]) -> Result<(), Self::Error>;
}

/// Async interface operations for `MS56xx` sensors.
#[allow(async_fn_in_trait)]
pub trait Ms56xxInterfaceAsync {
    type Error;

    /// Write a command byte.
    ///
    /// # Errors
    /// Returns an error if the interface communication fails.
    async fn write_cmd(&mut self, cmd: u8) -> Result<(), Self::Error>;

    /// Write a command byte and read the response.
    ///
    /// # Errors
    /// Returns an error if the interface communication fails.
    async fn read(&mut self, cmd: u8, buf: &mut [u8]) -> Result<(), Self::Error>;
}

/// I2C interface adapter for `MS56xx` sensors.
pub struct I2cInterface<I2C> {
    i2c: I2C,
    addr: u8,
}

impl<I2C> I2cInterface<I2C> {
    /// Create a new I2C interface adapter with the given address.
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self { i2c, addr }
    }

    /// Release the underlying I2C bus.
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    /// Get the configured I2C address.
    pub fn addr(&self) -> u8 {
        self.addr
    }
}

impl<I2C: BlockingI2c> Ms56xxInterface for I2cInterface<I2C> {
    type Error = I2C::Error;

    fn write_cmd(&mut self, cmd: u8) -> Result<(), Self::Error> {
        self.i2c.write(self.addr, &[cmd])
    }

    fn read(&mut self, cmd: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.addr, &[cmd], buf)
    }
}

impl<I2C: AsyncI2c> Ms56xxInterfaceAsync for I2cInterface<I2C> {
    type Error = I2C::Error;

    async fn write_cmd(&mut self, cmd: u8) -> Result<(), Self::Error> {
        self.i2c.write(self.addr, &[cmd]).await
    }

    async fn read(&mut self, cmd: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.addr, &[cmd], buf).await
    }
}

/// SPI interface adapter for `MS56xx` sensors (CS is handled by `SpiDevice`).
pub struct SpiInterface<SPI> {
    spi: SPI,
}

impl<SPI> SpiInterface<SPI> {
    /// Create a new SPI interface adapter.
    pub fn new(spi: SPI) -> Self {
        Self { spi }
    }

    /// Release the underlying SPI device.
    pub fn destroy(self) -> SPI {
        self.spi
    }
}

impl<SPI: BlockingSpiDevice> Ms56xxInterface for SpiInterface<SPI> {
    type Error = SPI::Error;

    fn write_cmd(&mut self, cmd: u8) -> Result<(), Self::Error> {
        self.spi.write(&[cmd])
    }

    fn read(&mut self, cmd: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        use embedded_hal::spi::Operation;
        self.spi
            .transaction(&mut [Operation::Write(&[cmd]), Operation::Read(buf)])
    }
}

impl<SPI: AsyncSpiDevice> Ms56xxInterfaceAsync for SpiInterface<SPI> {
    type Error = SPI::Error;

    async fn write_cmd(&mut self, cmd: u8) -> Result<(), Self::Error> {
        self.spi.write(&[cmd]).await
    }

    async fn read(&mut self, cmd: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        use embedded_hal_async::spi::Operation;
        self.spi
            .transaction(&mut [Operation::Write(&[cmd]), Operation::Read(buf)])
            .await
    }
}

/// Typestate marker for MS5607 sensor.
pub struct Ms5607;
/// Typestate marker for MS5611 sensor.
pub struct Ms5611;
/// Typestate marker for MS5637 sensor.
pub struct Ms5637;

/// Compile-time gate for variants that support SPI.
pub trait SupportsSpi {}
impl SupportsSpi for Ms5607 {}
impl SupportsSpi for Ms5611 {}

/// Trait for sensor-variant-specific behavior.
///
/// Keep first-order constants here as requested; everything else in this trait is
/// variant-dependent and used by generic driver code.
pub trait SensorVariant {
    /// Oversampling set supported by this sensor.
    type Oversampling: OversamplingType;

    /// PROM words physically present (MS5607/MS5611: 8, MS5637: 7).
    const PROM_WORDS: usize;

    /// First-order formula shifts (variant-dependent).
    const OFF_SHIFT: u32;
    const OFF_DT_SHIFT: u32;
    const SENS_SHIFT: u32;
    const SENS_DT_SHIFT: u32;

    /// 7-bit I2C address selection.
    fn i2c_address(csb_high: bool) -> u8;

    /// Extract stored CRC nibble from PROM.
    fn stored_crc(prom: &[u16; 8]) -> u8;

    /// Modify PROM copy for CRC calculation (clear stored CRC bits and any required fields).
    fn scrub_crc_for_calc(prom: &mut [u16; 8]);

    /// Apply second-order temperature compensation.
    fn second_order(temp: i64, dt: i64) -> (i64, i64, i64); // (t2, off2, sens2)
}

impl SensorVariant for Ms5607 {
    type Oversampling = OversamplingStandard;

    const PROM_WORDS: usize = 8;

    const OFF_SHIFT: u32 = 17;
    const OFF_DT_SHIFT: u32 = 6;
    const SENS_SHIFT: u32 = 16;
    const SENS_DT_SHIFT: u32 = 7;

    fn i2c_address(csb_high: bool) -> u8 {
        if csb_high { 0x76 } else { 0x77 }
    }

    fn stored_crc(prom: &[u16; 8]) -> u8 {
        (prom[7] & 0x000F) as u8
    }

    fn scrub_crc_for_calc(prom: &mut [u16; 8]) {
        // Typical AN520/MS56xx CRC prep: clear CRC nibble in word 7 low bits.
        prom[7] &= 0xFF00;
    }

    fn second_order(temp: i64, dt: i64) -> (i64, i64, i64) {
        let mut t2 = 0i64;
        let mut off2 = 0i64;
        let mut sens2 = 0i64;

        if temp < 2000 {
            let t_low = temp - 2000;
            t2 = (dt * dt) >> 31;
            off2 = (61 * t_low * t_low) >> 4;
            sens2 = 2 * t_low * t_low;

            if temp < -1500 {
                let tvl = temp + 1500;
                off2 += 15 * tvl * tvl;
                sens2 += 8 * tvl * tvl;
            }
        }

        (t2, off2, sens2)
    }
}

impl SensorVariant for Ms5611 {
    type Oversampling = OversamplingStandard;

    const PROM_WORDS: usize = 8;

    // MS5611 differs from MS5607 here.
    const OFF_SHIFT: u32 = 16;
    const OFF_DT_SHIFT: u32 = 7;
    const SENS_SHIFT: u32 = 15;
    const SENS_DT_SHIFT: u32 = 8;

    fn i2c_address(csb_high: bool) -> u8 {
        if csb_high { 0x76 } else { 0x77 }
    }

    fn stored_crc(prom: &[u16; 8]) -> u8 {
        (prom[7] & 0x000F) as u8
    }

    fn scrub_crc_for_calc(prom: &mut [u16; 8]) {
        prom[7] &= 0xFF00;
    }

    fn second_order(temp: i64, dt: i64) -> (i64, i64, i64) {
        let t2;
        let mut off2 = 0i64;
        let mut sens2 = 0i64;

        if temp < 2000 {
            let t_low = temp - 2000;
            t2 = (dt * dt) >> 31;
            off2 = (5 * t_low * t_low) >> 1;
            sens2 = (5 * t_low * t_low) >> 2;

            if temp < -1500 {
                let tvl = temp + 1500;
                off2 += 7 * tvl * tvl;
                sens2 += (11 * tvl * tvl) >> 1;
            }
        } else {
            t2 = 0;
        }

        (t2, off2, sens2)
    }
}

impl SensorVariant for Ms5637 {
    type Oversampling = OversamplingExtended;

    // 112-bit PROM => 7 words.
    const PROM_WORDS: usize = 7;

    // MS5637 uses the MS5607-style first-order shifts.
    const OFF_SHIFT: u32 = 17;
    const OFF_DT_SHIFT: u32 = 6;
    const SENS_SHIFT: u32 = 16;
    const SENS_DT_SHIFT: u32 = 7;

    fn i2c_address(_csb_high: bool) -> u8 {
        // Fixed 7-bit address.
        0x76
    }

    fn stored_crc(prom: &[u16; 8]) -> u8 {
        ((prom[0] >> 12) & 0x000F) as u8
    }

    fn scrub_crc_for_calc(prom: &mut [u16; 8]) {
        // MS5637 datasheet CRC reference: clear CRC nibble in word0 and set word7=0.
        prom[0] &= 0x0FFF;
        prom[7] = 0;
    }

    fn second_order(temp: i64, dt: i64) -> (i64, i64, i64) {
        let t2;
        let mut off2 = 0i64;
        let mut sens2 = 0i64;

        if temp < 2000 {
            let t_low = temp - 2000;
            t2 = (3 * dt * dt) >> 33;
            off2 = (61 * t_low * t_low) >> 4;
            sens2 = (29 * t_low * t_low) >> 4;

            if temp < -1500 {
                let tvl = temp + 1500;
                off2 += 17 * tvl * tvl;
                sens2 += 9 * tvl * tvl;
            }
        } else {
            t2 = (5 * dt * dt) >> 38;
        }

        (t2, off2, sens2)
    }
}

/// A driver for `MS56xx` pressure sensors (MS5607, MS5611, MS5637).
pub struct Ms56xx<INTERFACE, VARIANT: SensorVariant> {
    interface: INTERFACE,
    prom: [u16; 8],
    initialized: bool,
    _variant: PhantomData<VARIANT>,
}

impl<I2C, VARIANT: SensorVariant> Ms56xx<I2cInterface<I2C>, VARIANT> {
    /// Create a new sensor driver using I2C.
    ///
    /// `csb_high` is used only by variants that have selectable I2C address.
    pub fn new_i2c(i2c: I2C, csb_high: bool) -> Self {
        let addr = VARIANT::i2c_address(csb_high);
        Self {
            interface: I2cInterface::new(i2c, addr),
            prom: [0; 8],
            initialized: false,
            _variant: PhantomData,
        }
    }

    /// Release the underlying I2C bus.
    pub fn destroy(self) -> I2C {
        self.interface.destroy()
    }

    /// Get the configured I2C address of the sensor.
    pub fn address(&self) -> u8 {
        self.interface.addr()
    }
}

impl<SPI, VARIANT: SensorVariant + SupportsSpi> Ms56xx<SpiInterface<SPI>, VARIANT> {
    /// Create a new sensor driver using SPI.
    ///
    /// Only available for variants that support SPI (compile-time gated).
    pub fn new_spi(spi: SPI) -> Self {
        Self {
            interface: SpiInterface::new(spi),
            prom: [0; 8],
            initialized: false,
            _variant: PhantomData,
        }
    }
}

impl<SPI, VARIANT: SensorVariant> Ms56xx<SpiInterface<SPI>, VARIANT> {
    /// Release the underlying SPI device.
    pub fn destroy(self) -> SPI {
        self.interface.destroy()
    }
}

impl<INTERFACE, VARIANT: SensorVariant> Ms56xx<INTERFACE, VARIANT> {
    /// Returns true if the sensor has been initialized successfully.
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Check the CRC4 of the PROM. Returns true if it matches the stored value.
    #[allow(arithmetic_overflow)]
    fn check_crc4(&self) -> bool {
        let stored_crc = VARIANT::stored_crc(&self.prom);
        let computed_crc = calculate_crc4::<VARIANT>(&self.prom);
        computed_crc == stored_crc
    }

    /// Convert raw D1 & D2 into compensated values (incl. variant second order).
    #[allow(clippy::cast_possible_truncation)]
    fn compensate(&self, d1: u32, d2: u32) -> (i32, i32) {
        let c1 = i64::from(self.prom[1]);
        let c2 = i64::from(self.prom[2]);
        let c3 = i64::from(self.prom[3]);
        let c4 = i64::from(self.prom[4]);
        let c5 = i64::from(self.prom[5]);
        let c6 = i64::from(self.prom[6]);

        let d1 = i64::from(d1);
        let d2 = i64::from(d2);

        let dt = d2 - (c5 << 8);
        let temp = 2000 + (dt * c6) / (1 << 23);

        let off = (c2 << VARIANT::OFF_SHIFT) + ((c4 * dt) >> VARIANT::OFF_DT_SHIFT);
        let sens = (c1 << VARIANT::SENS_SHIFT) + ((c3 * dt) >> VARIANT::SENS_DT_SHIFT);

        let (t2, off2, sens2) = VARIANT::second_order(temp, dt);

        let temp_corrected = temp - t2;
        let off_corrected = off - off2;
        let sens_corrected = sens - sens2;

        let p = (((d1 * sens_corrected) >> 21) - off_corrected) >> 15;
        (p as i32, temp_corrected as i32)
    }
}

impl<INTERFACE: Ms56xxInterfaceAsync, VARIANT: SensorVariant> Ms56xx<INTERFACE, VARIANT> {
    /// Send reset command and wait for internal PROM reload (~3ms).
    ///
    /// # Errors
    /// Returns an error if interface communication fails.
    pub async fn reset(&mut self, delay: &mut impl AsyncDelay) -> Result<(), INTERFACE::Error> {
        self.interface.write_cmd(CMD_RESET).await?;
        delay.delay_ms(RESET_DELAY_MS).await;
        Ok(())
    }

    /// Read all PROM words present for this variant (big-endian u16).
    #[allow(clippy::cast_possible_truncation)]
    async fn read_prom_all(&mut self) -> Result<(), INTERFACE::Error> {
        self.prom = [0; 8];
        for i in 0..(VARIANT::PROM_WORDS as u8) {
            let cmd = CMD_PROM_BASE + (i * 2);
            let mut buf = [0u8; 2];
            self.interface.read(cmd, &mut buf).await?;
            self.prom[i as usize] = u16::from_be_bytes(buf);
        }
        Ok(())
    }

    /// Initialize the sensor by resetting it, reading PROM data and verifying the CRC.
    ///
    /// # Errors
    /// Returns [`Error::BusError`] if interface communication fails, or [`Error::CrcMismatch`] if the
    /// PROM CRC check fails.
    pub async fn init(
        &mut self,
        delay: &mut impl AsyncDelay,
    ) -> Result<(), Error<INTERFACE::Error>> {
        self.reset(delay).await.map_err(Error::BusError)?;
        self.read_prom_all().await.map_err(Error::BusError)?;

        if !self.check_crc4() {
            return Err(Error::CrcMismatch);
        }

        self.initialized = true;
        Ok(())
    }

    async fn start_conversion(
        &mut self,
        conv: ConversionType,
        osr: VARIANT::Oversampling,
    ) -> Result<(), INTERFACE::Error> {
        let cmd = (match conv {
            ConversionType::D1 => CMD_CONVERT_D1,
            ConversionType::D2 => CMD_CONVERT_D2,
        }) | osr.cmd_value();

        self.interface.write_cmd(cmd).await
    }

    async fn read_adc(&mut self) -> Result<u32, INTERFACE::Error> {
        let mut buf = [0u8; 3];
        self.interface.read(CMD_ADC_READ, &mut buf).await?;
        Ok((u32::from(buf[0]) << 16) | (u32::from(buf[1]) << 8) | u32::from(buf[2]))
    }

    async fn wait_conversion(osr: VARIANT::Oversampling, delay: &mut impl AsyncDelay) {
        delay.delay_us(osr.delay_us()).await;
    }

    /// Carry out a complete measurement and conversion of pressure and temperature.
    ///
    /// # Errors
    /// Returns [`Error::NotInitialized`] if the sensor has not been initialized yet, or
    /// [`Error::BusError`] if interface communication fails.
    pub async fn measure(
        &mut self,
        oversampling: VARIANT::Oversampling,
        delay: &mut impl AsyncDelay,
    ) -> Result<Measurement, Error<INTERFACE::Error>> {
        if !self.initialized {
            return Err(Error::NotInitialized);
        }

        self.start_conversion(ConversionType::D1, oversampling)
            .await
            .map_err(Error::BusError)?;
        Self::wait_conversion(oversampling, delay).await;
        let d1_raw = self.read_adc().await.map_err(Error::BusError)?;

        self.start_conversion(ConversionType::D2, oversampling)
            .await
            .map_err(Error::BusError)?;
        Self::wait_conversion(oversampling, delay).await;
        let d2_raw = self.read_adc().await.map_err(Error::BusError)?;

        let (p_i, t_i) = self.compensate(d1_raw, d2_raw);

        #[allow(clippy::cast_precision_loss)]
        Ok(Measurement {
            pressure_mbar: p_i as f32 / 100.0,
            temperature_c: t_i as f32 / 100.0,
        })
    }
}

impl<INTERFACE: Ms56xxInterface, VARIANT: SensorVariant> Ms56xx<INTERFACE, VARIANT> {
    /// Send reset command and wait for internal PROM reload (~3ms).
    ///
    /// This is the blocking version of [`reset`](Self::reset).
    ///
    /// # Errors
    /// Returns an error if interface communication fails.
    pub fn reset_blocking(
        &mut self,
        delay: &mut impl BlockingDelay,
    ) -> Result<(), INTERFACE::Error> {
        self.interface.write_cmd(CMD_RESET)?;
        delay.delay_ms(RESET_DELAY_MS);
        Ok(())
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_prom_all_blocking(&mut self) -> Result<(), INTERFACE::Error> {
        self.prom = [0; 8];
        for i in 0..(VARIANT::PROM_WORDS as u8) {
            let cmd = CMD_PROM_BASE + (i * 2);
            let mut buf = [0u8; 2];
            self.interface.read(cmd, &mut buf)?;
            self.prom[i as usize] = u16::from_be_bytes(buf);
        }
        Ok(())
    }

    /// Initialize the sensor by resetting it, reading PROM data and verifying the CRC.
    ///
    /// This is the blocking version of [`init`](Self::init).
    ///
    /// # Errors
    /// Returns [`Error::BusError`] if interface communication fails, or [`Error::CrcMismatch`] if the
    /// PROM CRC check fails.
    pub fn init_blocking(
        &mut self,
        delay: &mut impl BlockingDelay,
    ) -> Result<(), Error<INTERFACE::Error>> {
        self.reset_blocking(delay).map_err(Error::BusError)?;
        self.read_prom_all_blocking().map_err(Error::BusError)?;

        if !self.check_crc4() {
            return Err(Error::CrcMismatch);
        }

        self.initialized = true;
        Ok(())
    }

    fn start_conversion_blocking(
        &mut self,
        conv: ConversionType,
        osr: VARIANT::Oversampling,
    ) -> Result<(), INTERFACE::Error> {
        let cmd = (match conv {
            ConversionType::D1 => CMD_CONVERT_D1,
            ConversionType::D2 => CMD_CONVERT_D2,
        }) | osr.cmd_value();

        self.interface.write_cmd(cmd)
    }

    fn read_adc_blocking(&mut self) -> Result<u32, INTERFACE::Error> {
        let mut buf = [0u8; 3];
        self.interface.read(CMD_ADC_READ, &mut buf)?;
        Ok((u32::from(buf[0]) << 16) | (u32::from(buf[1]) << 8) | u32::from(buf[2]))
    }

    fn wait_conversion_blocking(osr: VARIANT::Oversampling, delay: &mut impl BlockingDelay) {
        delay.delay_us(osr.delay_us());
    }

    /// Carry out a complete measurement and conversion of pressure and temperature.
    ///
    /// This is the blocking version of [`measure`](Self::measure).
    ///
    /// # Errors
    /// Returns [`Error::NotInitialized`] if the sensor has not been initialized yet, or
    /// [`Error::BusError`] if interface communication fails.
    pub fn measure_blocking(
        &mut self,
        oversampling: VARIANT::Oversampling,
        delay: &mut impl BlockingDelay,
    ) -> Result<Measurement, Error<INTERFACE::Error>> {
        if !self.initialized {
            return Err(Error::NotInitialized);
        }

        self.start_conversion_blocking(ConversionType::D1, oversampling)
            .map_err(Error::BusError)?;
        Self::wait_conversion_blocking(oversampling, delay);
        let d1_raw = self.read_adc_blocking().map_err(Error::BusError)?;

        self.start_conversion_blocking(ConversionType::D2, oversampling)
            .map_err(Error::BusError)?;
        Self::wait_conversion_blocking(oversampling, delay);
        let d2_raw = self.read_adc_blocking().map_err(Error::BusError)?;

        let (p_i, t_i) = self.compensate(d1_raw, d2_raw);

        #[allow(clippy::cast_precision_loss)]
        Ok(Measurement {
            pressure_mbar: p_i as f32 / 100.0,
            temperature_c: t_i as f32 / 100.0,
        })
    }
}

/// Computes the CRC4 checksum for the PROM coefficients.
/// CRC placement and masking is variant-dependent.
#[allow(arithmetic_overflow)]
fn calculate_crc4<VARIANT: SensorVariant>(n_prom: &[u16; 8]) -> u8 {
    let mut copy = *n_prom;
    VARIANT::scrub_crc_for_calc(&mut copy);

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
    ((n_rem >> 12) & 0x000F) as u8
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc4_ms5607_vectors() {
        let n_prom: [u16; 8] = [
            0x3132, 0x3334, 0x3536, 0x3738, 0x3940, 0x4142, 0x4344, 0x4500,
        ];
        let crc = calculate_crc4::<Ms5607>(&n_prom);
        assert_eq!(crc, 0xB, "CRC mismatch: expected 0xB, got 0x{:X}", crc);

        let n_prom: [u16; 8] = [
            0x3132, 0x3334, 0x3536, 0x3738, 0x3940, 0x4142, 0x4344, 0x450B,
        ];
        let crc = calculate_crc4::<Ms5607>(&n_prom);
        let stored_crc = <Ms5607 as SensorVariant>::stored_crc(&n_prom);
        assert_eq!(
            crc, stored_crc,
            "CRC mismatch: expected 0xB, got 0x{:X}",
            crc
        );
    }

    #[test]
    fn test_crc_extract_ms5637() {
        let n_prom: [u16; 8] = [
            0xA000, // CRC nibble=0xA in upper nibble
            0x3334, 0x3536, 0x3738, 0x3940, 0x4142, 0x4344, 0x0000,
        ];
        let stored_crc = <Ms5637 as SensorVariant>::stored_crc(&n_prom);
        assert_eq!(stored_crc, 0xA);
    }
}
