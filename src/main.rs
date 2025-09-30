#![no_std]
#![no_main]

use wex_robot::prelude::*;

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let mut robot = Robot::init();
    robot.set_led_0(Level::High);

    loop {
        robot.set_led_1(Level::High);
        // for i in 0..=100i8 {
        //     robot.set_speeds(i, i);
        //     Timer::after_millis(10).await;
        // }

        robot.set_speeds(25, 25);
        Timer::after_secs(4).await;
        robot.set_speeds(50, 50);
        Timer::after_secs(4).await;
        robot.set_speeds(75, 75);
        Timer::after_secs(4).await;
        robot.set_speeds(100, 100);

        Timer::after_secs(10).await;
        robot.set_speeds(0, 0);

        robot.set_led_1(Level::Low);
        robot.set_led_2(Level::Low);
        robot.set_led_3(Level::Low);

        // robot.disable_boost();
        Timer::after_secs(10).await;
    }
}
