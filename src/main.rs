#![no_std]
#![no_main]

use wex_robot::prelude::*;
use wex_robot::COMMAND_SIGNAL as CHANNEL;

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let mut robot = Robot::init(_spawner, "MyRobot").await;

    let mut state = Level::Low;

    loop {
        state = match state {
            Level::Low => Level::High,
            Level::High => Level::Low,
        };
        robot.set_led_5(state);

        match CHANNEL.wait().await {
            b'w' => robot.set_speeds(50, 50),
            b'a' => robot.set_speeds(0, 25),
            b's' => robot.set_speeds(-50, -50),
            b'd' => robot.set_speeds(25, 0),
            b'x' => robot.set_speeds(0, 0),
            b'b' => robot.halt_motors(),
            _ => (),
        }

        // for i in 0..=100i8 {
        //     robot.set_speeds(i, i);
        //     Timer::after_millis(100).await;
        // }
        // Timer::after_secs(10).await;
        // for i in (0..=100i8).rev() {
        //     robot.set_speeds(i, i);
        //     Timer::after_millis(100).await;
        // }
    }
}
