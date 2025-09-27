#![no_std]

use assign_resources::assign_resources;
pub use defmt_rtt as _;
use embassy_rp::{
    gpio::{Level, Output},
    peripherals,
};
pub use panic_probe as _;
pub struct Robot<'a> {
    left_motor: Drv8838<'a>,
    right_motor: Drv8838<'a>,
}

assign_resources! {
     cyw43: CYW43Resources {
        pwr: PIN_23,
        cs: PIN_25,
        sck: PIN_24,
        mosi: PIN_29,
        pio: PIO0,
        dma: DMA_CH0
     },
     left_driver: LeftMotorDriver {
        sleep: PIN_18,
        phase: PIN_17,
        enable: PIN_16,
     },
     right_driver: RightMotorDriver {
        sleep: PIN_19,
        phase: PIN_20,
        enable: PIN_21,
     }
}

impl<'a> Robot<'a> {
    pub fn init() -> Self {
        let p = embassy_rp::init(Default::default());
        let r = split_resources!(p);

        Self {
            left_motor: Drv8838::new(
                Output::new(r.left_driver.sleep, Level::Low),
                Output::new(r.left_driver.phase, Level::Low),
                Output::new(r.left_driver.enable, Level::Low),
            ),
            right_motor: Drv8838::new(
                Output::new(r.right_driver.sleep, Level::Low),
                Output::new(r.right_driver.phase, Level::Low),
                Output::new(r.right_driver.enable, Level::Low),
            ),
        }
    }

    pub fn set_speeds(&mut self, left_motor: i8, right_motor: i8) {
        self.left_motor.set_speed(left_motor);
        self.right_motor.set_speed(right_motor);
    }
}

/// # DRV8338 Motor Driver
/// The DRV883x family of devices provides an
/// integrated motor driver solution for cameras,
/// consumer products, toys, and other low-voltage or
/// battery-powered motion control applications. The
/// device can drive one dc motor or other devices
/// like solenoids. The output driver block consists of N
///channel power MOSFETs configured as an H-bridge
/// to drive the motor winding. An internal charge pump
/// generates needed gate drive voltages
///
/// ## Device Logic
///
/// | nSleep | Phase | Enable | Out 1 | Out 2 | Function |
/// | --- | --- | --- | --- | --- | --- |
/// | 0 | x | x | Z | Z | Coast |
/// | 1 | x | 0 | L | L | Break |
/// | 1 | 1 | 1 | L | H | Reverse |
/// | 1 | 0 | 1 | H | L | Forward |
///
///
pub struct Drv8838<'a> {
    sleep: Output<'a>,
    phase: Output<'a>,
    enable: Output<'a>,
}

impl<'a> Drv8838<'a> {
    pub fn new(sleep: Output<'a>, phase: Output<'a>, enable: Output<'a>) -> Self {
        Self {
            sleep,
            phase,
            enable,
        }
    }

    pub fn set_speed(&mut self, speed: i8) {
        match speed {
            0 => {
                self.coast();
            }
            i8::MIN..0 => {
                self.phase.set_high();
                self.sleep.set_high();
            }
            0..=i8::MAX => {
                self.phase.set_low();
                self.sleep.set_high();
            }
        }

        // todo: set the enable speed
        self.enable.set_high();
    }

    pub fn motor_break(&mut self) {
        self.sleep.set_high();
        self.enable.set_low();
    }

    pub fn coast(&mut self) {
        self.sleep.set_low();
        self.enable.set_low();
    }
}
