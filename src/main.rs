#![no_std]
#![no_main]

use wex_robot::prelude::*;

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let mut robot = Robot::init(_spawner).await;

    let mut state = Level::Low;

    loop {
        state = match state {
            Level::Low => Level::High,
            Level::High => Level::Low,
        };
        robot.set_led_5(state);

        for i in 0..=100i8 {
            robot.set_speeds(i, i);
            Timer::after_millis(100).await;
        }
        Timer::after_secs(10).await;
        for i in (0..=100i8).rev() {
            robot.set_speeds(i, i);
            Timer::after_millis(100).await;
        }
        // Timer::after_secs(5).await;
        // for i in 0..=100i8 {
        //     robot.set_speeds(-i, -i);
        //     Timer::after_millis(100).await;
        // }
        // Timer::after_secs(5).await;
        // for i in (0..=100i8).rev() {
        //     robot.set_speeds(-i, -i);
        //     Timer::after_millis(100).await;
        // }
        // Timer::after_secs(5).await;

        // robot.set_speeds(0, 0);

        // Timer::after_secs(10).await;
    }
}
