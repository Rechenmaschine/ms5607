use crate::OversamplingType;

/// Typestate marker for MS5607 sensor.
pub struct Ms5607;
/// Typestate marker for MS5611 sensor.
pub struct Ms5611;
/// Typestate marker for MS5637 sensor.
pub struct Ms5637;

/// Marker trait for variants that support SPI.
pub(crate) trait SupportsSpi {}
impl SupportsSpi for Ms5607 {}
impl SupportsSpi for Ms5611 {}

mod sealed {
    pub trait Sealed {}
    impl Sealed for super::Ms5607 {}
    impl Sealed for super::Ms5611 {}
    impl Sealed for super::Ms5637 {}
}

/// Trait for sensor-variant-specific behavior.
///
/// This trait is sealed and cannot be implemented outside this crate.
pub trait SensorVariant: sealed::Sealed {
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
    type Oversampling = crate::OversamplingStandard;

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
    type Oversampling = crate::OversamplingStandard;

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
    type Oversampling = crate::OversamplingExtended;

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
