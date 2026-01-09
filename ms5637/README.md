# ms5637

`no_std` driver for the TE Connectivity MS5637 barometric pressure sensor.

- Supports I2C interface (fixed address 0x76)
- Async and blocking APIs via `embedded-hal` traits
- Extended oversampling range (256-8192)
- Second-order temperature compensation
- CRC validation of calibration data

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
ms5637 = "0.1"
```

### Example

```rust
use ms5637::{Ms5637, Oversampling};

// I2C (fixed address 0x76)
let mut sensor = Ms5637::new_i2c(i2c);

// Initialize and measure (async)
sensor.init(&mut delay).await?;
let measurement = sensor.measure(Oversampling::Osr4096, &mut delay).await?;

// Or use blocking API
sensor.init_blocking(&mut delay)?;
let measurement = sensor.measure_blocking(Oversampling::Osr8192, &mut delay)?;

println!("Pressure: {} mbar, Temp: {} °C",
    measurement.pressure_mbar, measurement.temperature_c);
```

## Cargo Features

- `defmt-03`: Enables `defmt::Format` for all public types

## Related Crates

- [`ms5607`](https://crates.io/crates/ms5607) - Driver for the MS5607 sensor
- [`ms5611`](https://crates.io/crates/ms5611) - Driver for the MS5611 sensor

## License

MIT or Apache-2.0 license, at your option.
