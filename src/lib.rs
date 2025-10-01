#![no_std]

use assign_resources::assign_resources;
use bt_hci::cmd::le::LeSetScanParams;
use bt_hci::controller::ControllerCmdSync;
use cyw43_pio::{PioSpi, RM2_CLOCK_DIVIDER};
pub use defmt_rtt as _;
use embassy_rp::{
    bind_interrupts,
    gpio::{Level, Output},
    peripherals::{self, DMA_CH0, PIO0},
    pio::{InterruptHandler, Pio},
    pwm::{Pwm, SetDutyCycle},
};
pub use panic_probe as _;
use static_cell::StaticCell;
use trouble_host::{
    prelude::{DefaultPacketPool, ExternalController},
    Address, HostResources,
};
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

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

/// Max number of connections
const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 1;

impl<'a> Robot<'a> {
    pub async fn init(spawner: embassy_executor::Spawner) -> Self {
        let p = embassy_rp::init(Default::default());
        let r = split_resources!(p);

        let desired_freq_hz = 25_000;
        let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();
        let divider = 16u8;
        let period = (clock_freq_hz / (desired_freq_hz * divider as u32)) as u16 - 1;

        let mut pwm_config = embassy_rp::pwm::Config::default();
        pwm_config.top = period;
        pwm_config.divider = divider.into();

        let (fw, clm, btfw) = {
            // IMPORTANT
            //
            // Download and make sure these files from https://github.com/embassy-rs/embassy/tree/main/cyw43-firmware
            // are available in `./examples/rp-pico-2-w`. (should be automatic)
            //
            // IMPORTANT
            let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
            let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");
            let btfw = include_bytes!("../cyw43-firmware/43439A0_btfw.bin");
            (fw, clm, btfw)
        };

        let pwr = Output::new(r.cyw43.pwr, Level::Low);
        let cs = Output::new(r.cyw43.cs, Level::High);
        let mut pio = Pio::new(r.cyw43.pio, Irqs);
        let spi = PioSpi::new(
            &mut pio.common,
            pio.sm0,
            RM2_CLOCK_DIVIDER,
            pio.irq0,
            cs,
            r.cyw43.mosi,
            r.cyw43.sck,
            r.cyw43.dma,
        );

        static STATE: StaticCell<cyw43::State> = StaticCell::new();
        let state = STATE.init(cyw43::State::new());
        let (_net_device, bt_device, mut control, runner) =
            cyw43::new_with_bluetooth(state, pwr, spi, fw, btfw).await;
        spawner.spawn(cyw43_task(runner)).unwrap();
        control.init(clm).await;

        let controller: ExternalController<_, 10> = ExternalController::new(bt_device);
        // ble_bas_peripheral::run(controller).await;

        let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
            HostResources::new();
        let address: Address = Address::random([0xff, 0x8f, 0x1b, 0x05, 0xe4, 0xff]);
        let stack = trouble_host::new(controller, &mut resources).set_random_address(address);

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
