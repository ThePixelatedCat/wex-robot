use std::error::Error;
use std::io::{self, Read, Write};
use std::time::Duration;
use uuid::Uuid;

use btleplug::api::{
    Central, CentralEvent, Characteristic, Manager as _, Peripheral as _, ScanFilter, WriteType
};
use btleplug::platform::Manager;
use futures::stream::StreamExt;
use tokio::time::{self, Instant};

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // UUIDs must match the peripheral code!
    let service_uuid = Uuid::parse_str("0000097d-0000-1000-8000-00805f9b34fb")?;
    let command_char_uuid = Uuid::parse_str("5cc11628-0528-4edb-af0a-5db2a02d6827")?;

    // Get BLE adapter
    let manager = Manager::new().await?;
    let adapters = manager.adapters().await?;
    let adapter = adapters
        .into_iter()
        .next()
        .expect("No Bluetooth adapters found");

    println!("Scanning for robot...");
    adapter.start_scan(Default::default()).await?;

    // Wait and look for peripheral with our service
    let peripheral = 'outer: loop {
        if let Ok(peripherals) = adapter.peripherals().await {
            for p in peripherals {
                println!("xx: {:?}", p);
                if let Ok(Some(props)) = p.properties().await {
                    if props.services.contains(&service_uuid) {
                        println!("Found robot: {:?}", props.local_name);
                        break 'outer p; // break from the outer loop with p
                    }
                }
            }
        }
        time::sleep(Duration::from_secs(1)).await;
    };

    // Connect
    peripheral.connect().await?;
    println!("Connected to robot.");

    // Discover services
    peripheral.discover_services().await?;
    let chars = peripheral.characteristics();

    let command_char = chars
        .iter()
        .find(|c| c.uuid == command_char_uuid)
        .expect("Command characteristic not found")
        .clone();

    println!("Ready. Enter single-character commands (F, B, L, R, S). Ctrl+C to quit.");

   run(&peripheral, &command_char).await
}

use btleplug::api::{Peripheral as _};
use crossterm::event::{self, Event, KeyCode, KeyEventKind};

async fn run(
    peripheral: &impl btleplug::api::Peripheral,
    command_char: &btleplug::api::Characteristic,
) -> Result<(), Box<dyn std::error::Error>> {
    println!("Press keys to send commands (F/B/L/R/S). Press 'q' to quit.");

    // Track the currently held key
    let mut active_key: Option<char> = None;
    let mut last_sent = Instant::now();

    loop {
        // Check for key events
        if event::poll(Duration::from_millis(50))? {
            if let Event::Key(key_event) = event::read()? {
                match key_event.kind {
                    KeyEventKind::Press => {
                        if let KeyCode::Char(c) = key_event.code {
                            if c == 'q' {
                                println!("Quitting.");
                                break;
                            }
                            active_key = Some(c);
                            // Send immediately on press
                            send_cmd(peripheral, command_char, c).await?;
                            last_sent = Instant::now();
                        }
                    }
                    KeyEventKind::Release => {
                        if let KeyCode::Char(c) = key_event.code {
                            if Some(c) == active_key {
                                active_key = None; // stop sending
                            }
                        }
                    }
                    KeyEventKind::Repeat => {
                        // optional: ignore, since we handle auto-repeat ourselves
                    }
                }
            }
        }

        // If a key is held, repeat every 200ms
        if let Some(c) = active_key {
            if last_sent.elapsed() >= Duration::from_millis(200) {
                send_cmd(peripheral, command_char, c).await?;
                last_sent = Instant::now();
            }
        }
    }

    Ok(())
}

async fn send_cmd(
    peripheral: &impl btleplug::api::Peripheral,
    command_char: &btleplug::api::Characteristic,
    c: char,
) -> Result<(), Box<dyn std::error::Error>> {
    let byte = c as u8;
    peripheral
        .write(command_char, &[byte], WriteType::WithoutResponse)
        .await?;
    println!("Sent command: {}", c);
    Ok(())
}