#![no_std]
#![no_main]

use wex_robot::Robot;

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let mut robot = Robot::init();

    loop {}
}
