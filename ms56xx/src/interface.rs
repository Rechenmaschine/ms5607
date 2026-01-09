use embedded_hal::{i2c::I2c as BlockingI2c, spi::SpiDevice as BlockingSpiDevice};
use embedded_hal_async::{i2c::I2c as AsyncI2c, spi::SpiDevice as AsyncSpiDevice};

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

/// SPI interface adapter for `MS56xx` sensors. Note that this does not handle the chip select (CS).
/// The provided `SpiDevice` must manage CS as appropriate.
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
