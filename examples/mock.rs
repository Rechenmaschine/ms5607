use futures::executor::block_on;
use ms5607::{Ms5607, Oversampling};

fn main() {
    block_on(async_main());
}

async fn async_main() {
    let i2c_interface = mock::MockI2c;

    // The address of the MS5607 is determined by the CSB pin, but you can also specify it manually
    let mut ms5607 = Ms5607::new(i2c_interface, true);
    // let driver = Ms5607::new(i2c_interface, 0x76);

    // the delay type is usually supplied by your HAL or RTOS
    let mut delay = mock::Delay;

    // Initialize the driver
    ms5607
        .init(&mut delay)
        .await
        .expect("Failed to initialize MS5607 driver");

    // Perform a measurement with the desired oversampling rate
    let _ = ms5607
        .measure(Oversampling::Osr2048, &mut delay)
        .await
        .expect("Failed to perform async measurement");

    // We can also perform blocking measurements
    let meas = ms5607
        .measure_blocking(Oversampling::Osr4096, &mut delay)
        .expect("Failed to perform blocking measurement");

    println!(
        "Measured temp = {} °C, pressure = {} Pa",
        meas.temperature_c, meas.pressure_mbar
    );
}

mod mock {
    use embedded_hal::i2c::{ErrorType, Operation, SevenBitAddress};

    pub struct Delay;
    impl embedded_hal::delay::DelayNs for Delay {
        fn delay_ns(&mut self, _: u32) {}
    }
    impl embedded_hal_async::delay::DelayNs for Delay {
        async fn delay_ns(&mut self, _: u32) {}
    }

    pub struct MockI2c;
    #[derive(Debug)]
    pub struct MockI2cError;
    impl embedded_hal_async::i2c::Error for MockI2cError {
        fn kind(&self) -> embedded_hal_async::i2c::ErrorKind {
            embedded_hal_async::i2c::ErrorKind::Other
        }
    }
    impl ErrorType for MockI2c {
        type Error = MockI2cError;
    }
    impl embedded_hal_async::i2c::I2c for MockI2c {
        async fn transaction(
            &mut self,
            address: SevenBitAddress,
            operations: &mut [Operation<'_>],
        ) -> Result<(), Self::Error> {
            println!(
                "I2C Transaction to address {:x?} with operations: {:?}",
                address, operations
            );
            Ok(())
        }
    }
    impl embedded_hal::i2c::I2c for MockI2c {
        fn transaction(
            &mut self,
            address: SevenBitAddress,
            operations: &mut [Operation<'_>],
        ) -> Result<(), Self::Error> {
            println!(
                "I2C Transaction to address {:x?} with operations: {:?}",
                address, operations
            );
            Ok(())
        }
    }
}
