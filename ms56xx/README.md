# ms56xx

[![CI](https://github.com/Rechenmaschine/ms56xx/actions/workflows/rust.yml/badge.svg?branch=main)](https://github.com/Rechenmaschine/ms56xx/actions/workflows/rust.yml)
[![Docs.rs](https://img.shields.io/docsrs/ms56xx?logo=rust)](https://docs.rs/ms56xx)
[![Crates.io](https://img.shields.io/crates/v/ms56xx.svg)](https://crates.io/crates/ms56xx)

Internal implementation crate for `MS56xx` family barometric pressure sensors (MS5607, MS5611, MS5637).

Provides core driver logic with support for I2C and SPI interfaces, async and blocking APIs via `embedded-hal` traits, and compile-time sensor variant selection using the typestate pattern.

## For End Users

**Don't use this crate directly.** Instead, use one of the sensor-specific wrapper crates:

- [`ms5607-rs`](https://crates.io/crates/ms5607-rs) - For MS5607 sensors (I2C + SPI)
- [`ms5611-rs`](https://crates.io/crates/ms5611-rs) - For MS5611 sensors (I2C + SPI)
- [`ms5637-rs`](https://crates.io/crates/ms5637-rs) - For MS5637 sensors (I2C only)

These provide a more ergonomic API without exposing typestate implementation details.

## License

MIT or Apache-2.0 license, at your option.
