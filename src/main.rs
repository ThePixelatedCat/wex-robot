#![no_std]
#![no_main]

use wex_robot::prelude::*;

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    let mut robot = Robot::init(
        spawner,
        "MyRobot1",
        Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]),
    )
    .await;

    // ------------------- START MODIFYING HERE -----------------------
    // ----------------------------------------------------------------
}
