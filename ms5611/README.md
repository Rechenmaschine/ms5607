# ms5611

`no_std` driver for the TE Connectivity MS5611 barometric pressure sensor.

- Supports both I2C and SPI interfaces
- Async and blocking APIs via `embedded-hal` traits
- Second-order temperature compensation
- CRC validation of calibration data

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
ms5611 = "0.1"
```

### Example

```rust
use ms5611::{Ms5611, Oversampling};

// I2C (CSB pin high = address 0x76)
let mut sensor = Ms5611::new_i2c(i2c, true);

// Or SPI
let mut sensor = Ms5611::new_spi(spi);

// Initialize and measure (async)
sensor.init(&mut delay).await?;
let measurement = sensor.measure(Oversampling::Osr2048, &mut delay).await?;

// Or use blocking API
sensor.init_blocking(&mut delay)?;
let measurement = sensor.measure_blocking(Oversampling::Osr2048, &mut delay)?;

println!("Pressure: {} mbar, Temp: {} °C",
    measurement.pressure_mbar, measurement.temperature_c);
```

## Cargo Features

- `defmt-03`: Enables `defmt::Format` for all public types

## Related Crates

- [`ms5607`](https://crates.io/crates/ms5607) - Driver for the MS5607 sensor
- [`ms5637`](https://crates.io/crates/ms5637) - Driver for the MS5637 sensor

## License

MIT or Apache-2.0 license, at your option.
