# ms56xx

Internal implementation crate for MS56xx family barometric pressure sensors (MS5607, MS5611, MS5637).

Provides core driver logic with support for I2C and SPI interfaces, async and blocking APIs via `embedded-hal` traits

## Recommendation

**Don't use this crate directly.** Instead, use one of the sensor-specific wrapper crates:

- [`ms5607`](https://crates.io/crates/ms5607) - For MS5607 sensors (I2C + SPI)
- [`ms5611`](https://crates.io/crates/ms5611) - For MS5611 sensors (I2C + SPI)
- [`ms5637`](https://crates.io/crates/ms5637) - For MS5637 sensors (I2C only)

These provide a more ergonomic API without exposing typestate implementation details.

## License

MIT or Apache-2.0 license, at your option.
