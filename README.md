# MS56xx device drivers

[![CI](https://github.com/Rechenmaschine/ms56xx/actions/workflows/rust.yml/badge.svg?branch=main)](https://github.com/Rechenmaschine/ms56xx/actions/workflows/rust.yml)

> `no_std` drivers for the TE Connectivity MS56xx family of barometric pressure sensors.

- **Supports MS5607, MS5611, and MS5637** sensors
- **I2C and SPI** interfaces supported (SPI for MS5607/MS5611 only)
- **Async and blocking** APIs via `embedded-hal` traits
- Second-order temperature compensation
- CRC validation of calibration data

The MS56xx drivers are actively used in several projects at <https://github.com/aris-space> and deployed on flight hardware.

## Crates

- **[`ms5607-rs`](https://crates.io/crates/ms5607-rs)** - Driver for the MS5607 sensor
- **[`ms5611-rs`](https://crates.io/crates/ms5611-rs)** - Driver for the MS5611 sensor
- **[`ms5637-rs`](https://crates.io/crates/ms5637-rs)** - Driver for the MS5637 sensor

## Supported Sensors

- **MS5607-02BA03** - Datasheet: <https://www.amsys-sensor.com/downloads/data/MS5607-02BA03-AMSYS-datasheet.pdf>
- **MS5611-01BA03** - Datasheet: <https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS5611-01BA03%7FB3%7Fpdf%7FEnglish%7FENG_DS_MS5611-01BA03_B3.pdf>
- **MS5637-02BA03** - I2C only, extended oversampling range (256-8192)

All sensors use the same communication protocol but differ in their temperature compensation algorithms and features.

## Usage Examples

### Using the MS5607 crate

```rust
use ms5607_rs::{Ms5607, Oversampling};

// I2C (CSB pin high = address 0x76)
let mut sensor = Ms5607::new_i2c(i2c, true);

// Or SPI
let mut sensor = Ms5607::new_spi(spi);

// Async API
sensor.init(&mut delay).await?;
let measurement = sensor.measure(Oversampling::Osr2048, &mut delay).await?;

// Blocking API
sensor.init_blocking(&mut delay)?;
let measurement = sensor.measure_blocking(Oversampling::Osr2048, &mut delay)?;
```

### Using the MS5611 crate

```rust
use ms5611_rs::{Ms5611, Oversampling};

// Same API as MS5607
let mut sensor = Ms5611::new_i2c(i2c, true);
sensor.init(&mut delay).await?;
let measurement = sensor.measure(Oversampling::Osr4096, &mut delay).await?;
```

### Using the MS5637 crate

```rust
use ms5637_rs::{Ms5637, Oversampling};

// I2C only, fixed address 0x76
let mut sensor = Ms5637::new_i2c(i2c);
sensor.init(&mut delay).await?;
// Extended oversampling range
let measurement = sensor.measure(Oversampling::Osr8192, &mut delay).await?;
```

## Cargo Features

- `defmt-03`: Enables `defmt::Format` for all public types

## License

MIT or Apache-2.0 license, at your option.
