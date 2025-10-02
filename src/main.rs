#![no_std]
#![no_main]

use wex_robot::prelude::*;

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let mut robot = Robot::init(
        _spawner,
        "MyRobot1",
        Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]),
    )
    .await;

    let mut state = Level::Low;

    loop {
        state = match state {
            Level::Low => Level::High,
            Level::High => Level::Low,
        };
        robot.set_led_5(state);

        match COMMAND_SIGNAL.wait().await {
            b'w' => robot.set_speeds(50, 50),
            b'a' => robot.set_speeds(0, 25),
            b's' => robot.set_speeds(-50, -50),
            b'd' => robot.set_speeds(25, 0),
            b'x' => robot.set_speeds(0, 0),
            b'b' => robot.halt_motors(),
            KILL => robot.halt_motors(),
            _ => (),
        }
    }
}
