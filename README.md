# ms5607 device driver

[![CI](https://github.com/Rechenmaschine/ms5607/actions/workflows/rust.yml/badge.svg?branch=main)](https://github.com/Rechenmaschine/ms5607/actions/workflows/rust.yml)
[![Docs.rs](https://img.shields.io/docsrs/ms5607?logo=rust)](https://docs.rs/ms5607)
[![Crates.io](https://img.shields.io/crates/v/ms5607.svg)](https://crates.io/crates/ms5607)

`no_std` driver for the TE Connectivity MS5607 barometric pressure sensor. Supports both async and blocking operation using the `embedded-hal` traits.

Datasheet: <https://www.amsys-sensor.com/downloads/data/MS5607-02BA03-AMSYS-datasheet.pdf>

This driver is actively used in several projects at <https://github.com/aris-space> and is deployed on flight hardware.

## Usage
Both async and blocking APIs may be used at the same time provided the interface supports it.

**Async API**:
```rust
let mut ms5607 = Ms5607::new(i2c, true);
ms5607.init(&mut delay).await?;
let meas = ms5607.measure(Oversampling::Osr2048, &mut delay).await?;
```

**Blocking API**:
```rust
let mut ms5607 = Ms5607::new(i2c, true);
ms5607.init_blocking(&mut delay)?;
let meas = ms.measure_blocking(Oversampling::Osr2048, &mut delay)?;
```

## Features

- `defmt-03`: implements `defmt::Format` for all public types.

## License

MIT or Apache-2.0 license, at your option.
