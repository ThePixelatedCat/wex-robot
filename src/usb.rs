//! Implements the USB command channel

// use crate::STATE;

// use super::{Command, COMMAND_CHANNEL};
use embassy_executor::{SpawnError, Spawner};
use embassy_rp::{peripherals, usb, Peri};
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, State},
    UsbDevice,
};
pub use embassy_usb_logger::ReceiverHandler;
use heapless::String;
use static_cell::StaticCell;

pub type ObegransadUsbDriver = usb::Driver<'static, peripherals::USB>;
pub type ObegransadUsbDevice = UsbDevice<'static, ObegransadUsbDriver>;

/// Start all USB channels
///  - CommandChannel
///  - DefmtChannel
pub fn usb_start(spawner: &Spawner, p: Peri<'static, peripherals::USB>) -> Result<(), SpawnError> {
    let usb_driver = usb::Driver::new(p, super::Irqs);
    let usb_config = {
        let mut config = embassy_usb::Config::new(0xc001, 0xc0de);
        config.manufacturer = Some("Cloud Console Pty. Ltd.");
        config.product = Some("Obegransad 2.0");
        config.serial_number = Some("00001");
        config
    };

    // Ignore me I was making a second logging channel
    // let defmt_log_class = {
    //     static STATE: StaticCell<State> = StaticCell::new();
    //     let state = STATE.init(State::new());
    //     CdcAcmClass::new(&mut usb_builder, state, 64)
    // };

    let mut usb_builder = {
        static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

        let usb_builder = embassy_usb::Builder::new(
            usb_driver,
            usb_config,
            CONFIG_DESCRIPTOR.init([0; 256]),
            BOS_DESCRIPTOR.init([0; 256]),
            &mut [],
            CONTROL_BUF.init([0; 64]),
        );
        usb_builder
    };

    let logger_class = {
        static STATE: StaticCell<State> = StaticCell::new();
        let state = STATE.init(State::new());
        CdcAcmClass::new(&mut usb_builder, state, 64)
    };

    // Run the USB device forever
    let usb = usb_builder.build();
    spawner.spawn(usb_task(usb)?);
    spawner.spawn(usb_logger_task(logger_class)?);

    Ok(())
}

struct UsbCmdHandler;

impl ReceiverHandler for UsbCmdHandler {
    async fn handle_data(&self, data: &[u8]) -> () {
        // Handle incoming data from the USB CDC ACM class.
        // For now, we just log it.
        if let Ok(data) = str::from_utf8(data) {
            let data = data.trim();
            match data {
                _ => {
                    log::warn!("Unknown command: {}", data);
                }
            }
        } else {
            log::warn!("Received non-UTF8 data");
        }
    }

    fn new() -> Self {
        Self
    }
}

#[embassy_executor::task]
async fn usb_logger_task(class: CdcAcmClass<'static, ObegransadUsbDriver>) {
    embassy_usb_logger::with_class!(1024, log::LevelFilter::Info, class, UsbCmdHandler).await;
}

#[embassy_executor::task]
async fn usb_task(mut usb: ObegransadUsbDevice) -> ! {
    usb.run().await
}
