#![no_std]

use assign_resources::assign_resources;
pub use defmt_rtt as _;
use embassy_rp::{
    gpio::{Level, Output},
    peripherals,
    pwm::{Pwm, SetDutyCycle},
};
pub use panic_probe as _;

pub mod prelude {
    pub use super::Robot;
    pub use embassy_rp::gpio::Level;
    pub use embassy_time::Timer;
}

pub struct Robot<'a> {
    left_motor: Drv8838<'a>,
    right_motor: Drv8838<'a>,
    boost_en: Output<'a>,
    led_0: Output<'a>,
    led_1: Output<'a>,
    led_2: Output<'a>,
    led_3: Output<'a>,
    led_4: Output<'a>,
    led_5: Output<'a>,
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
        slice: PWM_SLICE0
     },
     right_driver: RightMotorDriver {
        sleep: PIN_19,
        phase: PIN_20,
        enable: PIN_21,
        slice: PWM_SLICE2,
     },
     leds: Leds {
        led_0: PIN_0,
        led_1: PIN_1,
        led_2: PIN_2,
        led_3: PIN_3,
        led_4: PIN_15,
        led_5: PIN_14,
     },
     system: SystemPins {
        boot_enable: PIN_22,
        batt_sense: PIN_26,
     }
}

impl<'a> Robot<'a> {
    pub fn init() -> Self {
        let p = embassy_rp::init(Default::default());
        let r = split_resources!(p);

        let desired_freq_hz = 25_000;
        let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();
        let divider = 16u8;
        let period = (clock_freq_hz / (desired_freq_hz * divider as u32)) as u16 - 1;

        let mut pwm_config = embassy_rp::pwm::Config::default();
        pwm_config.top = period;
        pwm_config.divider = divider.into();

        Self {
            left_motor: Drv8838::new(
                Output::new(r.left_driver.sleep, Level::Low),
                Output::new(r.left_driver.phase, Level::Low),
                Pwm::new_output_a(
                    r.left_driver.slice,
                    r.left_driver.enable,
                    pwm_config.clone(),
                ),
            ),
            right_motor: Drv8838::new(
                Output::new(r.right_driver.sleep, Level::Low),
                Output::new(r.right_driver.phase, Level::Low),
                Pwm::new_output_b(r.right_driver.slice, r.right_driver.enable, pwm_config),
            ),
            boost_en: Output::new(r.system.boot_enable, Level::High),
            led_0: Output::new(r.leds.led_0, Level::Low),
            led_1: Output::new(r.leds.led_1, Level::Low),
            led_2: Output::new(r.leds.led_2, Level::Low),
            led_3: Output::new(r.leds.led_3, Level::Low),
            led_4: Output::new(r.leds.led_4, Level::High),
            led_5: Output::new(r.leds.led_5, Level::Low),
        }
    }

    pub fn enable_boost(&mut self) {
        self.boost_en.set_high();
    }

    pub fn disable_boost(&mut self) {
        self.boost_en.set_low();
    }

    pub fn set_led_0(&mut self, level: Level) {
        self.led_0.set_level(level);
    }

    pub fn set_led_1(&mut self, level: Level) {
        self.led_1.set_level(level);
    }

    pub fn set_led_2(&mut self, level: Level) {
        self.led_2.set_level(level);
    }

    pub fn set_led_3(&mut self, level: Level) {
        self.led_3.set_level(level);
    }

    pub fn set_led_4(&mut self, level: Level) {
        self.led_4.set_level(level);
    }

    pub fn set_led_5(&mut self, level: Level) {
        self.led_5.set_level(level);
    }

    pub fn set_speeds(&mut self, left_motor: i8, right_motor: i8) {
        self.left_motor.set_speed(-left_motor);
        self.right_motor.set_speed(right_motor);
    }

    pub fn halt_motors(&mut self) {
        self.left_motor.motor_break();
        self.right_motor.motor_break();
    }
}

/// # DRV8338 Motor Driver
/// The DRV883x family of devices provides an
/// integrated motor driver solution for cameras,
/// consumer products, toys, and other low-voltage or
/// battery-powered motion control applications. The
/// device can drive one dc motor or other devices
/// like solenoids. The output driver block consists of N
/// channel power MOSFETs configured as an H-bridge
/// to drive the motor winding. An internal charge pump
/// generates needed gate drive voltages
///
pub struct Drv8838<'a> {
    sleep: Output<'a>,
    phase: Output<'a>,
    enable: Pwm<'a>,
}

impl<'a> Drv8838<'a> {
    pub fn new(sleep: Output<'a>, phase: Output<'a>, enable: Pwm<'a>) -> Self {
        Self {
            sleep,
            phase,
            enable,
        }
    }

    /// Set the output speed as a percentage
    /// If zero, driver will coast, negative numbers will reverse drive
    ///
    /// ## Device Logic
    ///
    /// | nSleep | Phase | Enable | Out 1 | Out 2 | Function |
    /// | --- | --- | --- | --- | --- | --- |
    /// | 0 | x | x | Z | Z | Coast |
    /// | 1 | x | 0 | L | L | Break |
    /// | 1 | 1 | 1 | L | H | Reverse |
    /// | 1 | 0 | 1 | H | L | Forward |
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

        let _ = self.enable.set_duty_cycle_percent(speed.abs() as u8);
    }

    pub fn motor_break(&mut self) {
        self.sleep.set_high();
        let _ = self.enable.set_duty_cycle_fully_off();
    }

    pub fn coast(&mut self) {
        self.sleep.set_low();
        let _ = self.enable.set_duty_cycle_fully_off();
    }
}
