#![no_std]

use assign_resources::assign_resources;
use bt_hci::controller::ControllerCmdSync;
use bt_hci::{cmd::le::LeSetScanParams, transport::Transport};
use cyw43::bluetooth::BtDriver;
use cyw43_pio::{PioSpi, RM2_CLOCK_DIVIDER};
pub use defmt_rtt as _;
use embassy_futures::join::join;
use embassy_futures::select::select;
use embassy_rp::{
    bind_interrupts,
    gpio::{Level, Output},
    peripherals::{self, DMA_CH0, PIO0, USB},
    pio::{InterruptHandler, Pio},
    pwm::{Pwm, SetDutyCycle},
    usb::InterruptHandler as UsbInterruptHandler,
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, signal::Signal};
use embassy_time::Timer;
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
mod usb;

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

// assign_resources! {
//      cyw43: CYW43Resources {
//         pwr: PIN_23,
//         cs: PIN_25,
//         sck: PIN_24,
//         mosi: PIN_29,
//         pio: PIO0,
//         dma: DMA_CH0
//      },
//      left_driver: LeftMotorDriver {
//         sleep: PIN_18,
//         phase: PIN_17,
//         enable: PIN_16,
//         slice: PWM_SLICE0
//      },
//      right_driver: RightMotorDriver {
//         sleep: PIN_19,
//         phase: PIN_20,
//         enable: PIN_21,
//         slice: PWM_SLICE2,
//      },
//      leds: Leds {
//         led_0: PIN_0,
//         led_1: PIN_1,
//         led_2: PIN_2,
//         led_3: PIN_3,
//         led_4: PIN_15,
//         led_5: PIN_14,
//      },
//      system: SystemPins {
//         boot_enable: PIN_22,
//         batt_sense: PIN_26,
//      }
// }

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
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

use trouble_host::prelude::*;

// GATT Server definition
#[gatt_server]
struct Server {
    service: RobotService,
}

#[gatt_service(uuid = "0000097d-0000-1000-8000-00805f9b34fb")]
struct RobotService {
    /// Write-only command characteristic
    #[characteristic(uuid = "5cc11628-0528-4edb-af0a-5db2a02d6827", write, read)]
    command: u8,
}

pub type Command = u8;
pub type CommandSignal = Signal<ThreadModeRawMutex, Command>;
pub static COMMAND_SIGNAL: CommandSignal = Signal::new();

impl<'a> Robot<'a> {
    pub async fn init(spawner: embassy_executor::Spawner, name: &'static str) -> Self {
        let p = embassy_rp::init(Default::default());

        let desired_freq_hz = 25_000;
        let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();
        let divider = 16u8;
        let period = (clock_freq_hz / (desired_freq_hz * divider as u32)) as u16 - 1;

        let mut pwm_config = embassy_rp::pwm::Config::default();
        pwm_config.top = period;
        pwm_config.divider = divider.into();

        let _ = usb::usb_start(&spawner, p.USB);
        info!("Hello World");
        Timer::after_secs(3).await;

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

        let pwr = Output::new(p.PIN_23, Level::Low);
        let cs = Output::new(p.PIN_25, Level::High);
        let mut pio = Pio::new(p.PIO0, Irqs);
        let spi = PioSpi::new(
            &mut pio.common,
            pio.sm0,
            RM2_CLOCK_DIVIDER,
            pio.irq0,
            cs,
            p.PIN_24,
            p.PIN_29,
            p.DMA_CH0,
        );

        info!("Starting Bluetooth");
        static STATE: StaticCell<cyw43::State> = StaticCell::new();
        let state = STATE.init(cyw43::State::new());
        let (_net_device, bt_device, mut control, runner) =
            cyw43::new_with_bluetooth(state, pwr, spi, fw, btfw).await;
        info!("Spawning BT runner");
        spawner.spawn(cyw43_task(runner).unwrap());
        control.init(clm).await;

        let controller: ExternalController<BtDriver, 10> = ExternalController::new(bt_device);

        // ---------- START MESSING AROUND ----------

        spawner.spawn(bt_listener(controller, name).unwrap());

        // ---------- END MESSING AROUND ----------

        Self {
            left_motor: Drv8838::new(
                Output::new(p.PIN_18, Level::Low),
                Output::new(p.PIN_17, Level::Low),
                Pwm::new_output_a(p.PWM_SLICE0, p.PIN_16, pwm_config.clone()),
            ),
            right_motor: Drv8838::new(
                Output::new(p.PIN_19, Level::Low),
                Output::new(p.PIN_20, Level::Low),
                Pwm::new_output_b(p.PWM_SLICE2, p.PIN_21, pwm_config),
            ),
            boost_en: Output::new(p.PIN_22, Level::High),
            led_0: Output::new(p.PIN_0, Level::Low),
            led_1: Output::new(p.PIN_1, Level::Low),
            led_2: Output::new(p.PIN_2, Level::Low),
            led_3: Output::new(p.PIN_3, Level::Low),
            led_4: Output::new(p.PIN_15, Level::High),
            led_5: Output::new(p.PIN_14, Level::Low),
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

#[embassy_executor::task]
async fn bt_listener(
    controller: ExternalController<BtDriver<'static>, 10>,
    name: &'static str,
) -> ! {
    run(controller, name).await;
}

/// Run the BLE stack.
pub async fn run<C>(controller: C, name: &str) -> !
where
    C: Controller,
{
    // Using a fixed "random" address can be useful for testing. In real scenarios, one would
    // use e.g. the MAC 6 byte array as the address (how to get that varies by the platform).
    let address: Address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);
    // info!("Our address = {:?}", address);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        runner,
        ..
    } = stack.build();

    // info!("Starting advertising and GATT service");
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name,
        appearance: &appearance::UNKNOWN,
    }))
    .unwrap();

    let _ = join(ble_task(runner), async {
        loop {
            match advertise(name, &mut peripheral, &server).await {
                Ok(conn) => {
                    // set up tasks when the connection is established to a central, so they don't run when no one is connected.
                    let a = gatt_events_task(&server, &conn);
                    a.await;
                    // let b = custom_task(&server, &conn, &stack);
                    // // run until any task ends (usually because the connection has been closed),
                    // // then return to advertising state.
                    // select(a, b).await;
                }
                Err(e) => {
                    panic!("[adv] error: {:?}", e);
                }
            }
        }
    })
    .await;

    unreachable!()
}

/// This is a background task that is required to run forever alongside any other BLE tasks.
async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(e) = runner.run().await {
            panic!("[ble_task] error: {:?}", e);
        }
    }
}

/// Stream Events until the connection closes.
///
/// This function will handle the GATT events and process them.
/// This is how we interact with read and write requests.
async fn gatt_events_task<P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
) -> Result<(), Error> {
    let command = server.service.command;
    let reason = loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => break reason,
            GattConnectionEvent::Gatt { event } => {
                match &event {
                    // GattEvent::Read(event) => {
                    //     if event.handle() == command.handle {
                    //         let value = server.get(&command);
                    //         info!("[gatt] Read Event to Command Characteristic: {:?}", value);
                    //     }
                    // }
                    GattEvent::Write(event) => {
                        // HERE
                        if event.handle() == command.handle {
                            let data = event.data();
                            log::debug!("[gatt] Write Event to Command Characteristic: {:?}", data);
                            COMMAND_SIGNAL.signal(data[0]);
                        }
                    }
                    _ => {}
                };
                // This step is also performed at drop(), but writing it explicitly is necessary
                // in order to ensure reply is sent.
                match event.accept() {
                    Ok(reply) => reply.send().await,
                    Err(e) => log::warn!("[gatt] error sending response: {:?}", e),
                };
            }
            _ => {} // ignore other Gatt Connection Events
        }
    };
    info!("[gatt] disconnected: {:?}", reason);
    Ok(())
}

/// Create an advertiser to use to connect to a BLE Central, and wait for it to connect.
async fn advertise<'values, 'server, C: Controller>(
    name: &'values str,
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut advertiser_data = [0; 31];

    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids16(&[[0x7D, 0x09]]),
            AdStructure::CompleteLocalName(name.as_bytes()),
        ],
        &mut advertiser_data[..],
    )?;
    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &advertiser_data[..len],
                scan_data: &[],
            },
        )
        .await?;
    info!("[adv] advertising");
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    info!("[adv] connection established");
    Ok(conn)
}

/// Example task to use the BLE notifier interface.
/// This task will notify the connected central of a counter value every 2 seconds.
/// It will also read the RSSI value every 2 seconds.
/// and will stop when the connection is closed by the central or an error occurs.
async fn custom_task<C: Controller, P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
    stack: &Stack<'_, C, P>,
) {
    let mut tick: u8 = 0;
    // let command = server.service.command;
    loop {
        // tick = tick.wrapping_add(1);
        // info!("[custom_task] notifying connection of tick {}", tick);
        // if command.notify(conn, &tick).await.is_err() {
        //     info!("[custom_task] error notifying connection");
        //     break;
        // };
        // read RSSI (Received Signal Strength Indicator) of the connection.
        if let Ok(rssi) = conn.raw().rssi(stack).await {
            info!("[custom_task] RSSI: {:?}", rssi);
        } else {
            info!("[custom_task] error getting RSSI");
            break;
        };
        Timer::after_secs(2).await;
    }
}

use log::{error, info, warn};
