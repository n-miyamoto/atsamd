use atsamd_hal::adc::Adc;
use atsamd_hal::clock::GenericClockController;
//use atsamd_hal::gpio::{Floating, Input, Pa12, Pa13, Pa16, Pa17, Pd1, PfB, PfD, Port};
use atsamd_hal::gpio::v2::{Floating, Input, PA12, PA13, PA16, PA17};
use atsamd_hal::prelude::*;
//use atsamd_hal::sercom::{I2CMaster4, PadPin, Sercom4Pad0, Sercom4Pad1};
//use atsamd_hal::sercom::{I2CMaster3, Sercom3Pad0, Sercom3Pad1};
//use atsamd_hal::target_device::gclk::pchctrl::GEN_A::GCLK11;
//use atsamd_hal::target_device::{ADC1, MCLK, SERCOM3, SERCOM4};
use atsamd_hal::pac::gclk::pchctrl::GEN_A::GCLK11;
use atsamd_hal::pac::{ADC1, MCLK, SERCOM4, SERCOM3};
use atsamd_hal::sercom::{
    v2::{i2c, IoSet3, Sercom4, Sercom3},
//    PadPin,
};
use atsamd_hal::time::U32Ext;

use lis3dh::{Lis3dh, SlaveAddr};

use super::pins::aliases::*;

/// I2C Accelerometer pins (uses `SERCOM4`)
pub struct Accelerometer {
    /// `I2C0` bus clock pin
    pub scl: I2c0SclReset,

    /// `I2C0` bus data pin
    pub sda: I2c0SdaReset,
}

/// I2C pads for the labelled I2C peripheral
///
/// You can use these pads with other, user-defined [`i2c::Config`]urations.
pub type I2c0Pads = i2c::Pads<Sercom4, IoSet3, I2c0Sda, I2c0Scl>;

impl Accelerometer {
    /// Initialize the LIS3DH accelerometer using the correct pins and
    // peripherals. Use the driver's default settings.
    pub fn init(
        self,
        clocks: &mut GenericClockController,
        sercom4: SERCOM4,
        mclk: &mut MCLK,
    ) -> Lis3dh<i2c::I2c<i2c::Config<I2c0Pads>>> {
        // The accelerometer is connected to the Wio Terminal's `I2C0` bus, so
        // based on the possible padouts listed in the datasheet it must use
        // `SERCOM4`.
        let gclk0 = clocks.gclk0();
        let clock = &clocks.sercom4_core(&gclk0).unwrap();
        let freq = clock.freq();
        let (sda, scl): (I2c0Sda, I2c0Scl) = (self.sda.into(), self.scl.into());
        let pads: I2c0Pads = i2c::Pads::new(sda, scl);
        let i2c = i2c::Config::new(mclk, sercom4, pads, freq)
            .baud(400.khz())
            .enable();

        // The schematic states that the alternate I2C address `0x19` is used,
        // but that doesn't appear to work!
        Lis3dh::new(i2c, SlaveAddr::Default).unwrap()
    }
}

/// Analog Light Sensor
pub struct LightSensor {
    /// Analog Light Sensor input pin
    pub pd1: LightSensorAdcReset,
}

impl LightSensor {
    /// Initialize Pd1 as an ADC input, and return a Tuple containing the ADC
    /// peripheral and the configured pin.
    pub fn init(
        self,
        adc: ADC1,
        clocks: &mut GenericClockController,
        mclk: &mut MCLK,
    ) -> (Adc<ADC1>, LightSensorAdc) {
        let adc1 = Adc::adc1(adc, mclk, clocks, GCLK11);

        (adc1, self.pd1.into())
    }
}

/// general I2C pins (uses `SERCOM3`)
pub struct I2C{
//    /// `I2C1` bus clock pin
//    pub scl: PA16,
//
//    /// `I2C1` bus data pin
//    pub sda: PA17,
//    /// `I2C0` bus clock pin
    pub scl: I2c1SclReset,

    /// `I2C0` bus data pin
    pub sda: I2c1SdaReset,
}
/// I2C pads for the labelled I2C peripheral
///
/// You can use these pads with other, user-defined [`i2c::Config`]urations.
pub type I2c1Pads = i2c::Pads<Sercom3, IoSet3, I2c1Sda, I2c1Scl>;
impl I2C{
    pub fn init(
        self,
        clocks: &mut GenericClockController,
        sercom3: SERCOM3,
        mclk: &mut MCLK,
    ) -> i2c::I2c<i2c::Config<I2c1Pads>> {
        // I2C groove connector is connected to the Wio Terminal's `I2C1` bus, so
        // based on the possible padouts listed in the datasheet it must use
        let gclk0 = clocks.gclk0();
        // `SERCOM3`
        let clock = &clocks.sercom3_core(&gclk0).unwrap();
        let freq = clock.freq();
        let (sda, scl): (I2c1Sda, I2c1Scl) = (self.sda.into(), self.scl.into());
        let pads: I2c1Pads = i2c::Pads::new(sda, scl);
        let i2c = i2c::Config::new(mclk, sercom3, pads, freq)
            .baud(400.khz())
            .enable();
        
        //let gclk0 = clocks.gclk0();
        //let i2c = I2CMaster3::new(
        //    &clocks.sercom3_core(&gclk0).unwrap(),
        //    400.khz(),
        //    sercom3,
        //    mclk,
        //    self.sda.into_pad(port),
        //    self.scl.into_pad(port),
        //);

        i2c
    }
}
