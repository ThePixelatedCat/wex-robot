#![no_std]
#![no_main]

use wex_robot::prelude::*;

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    let mut robot = Robot::init(
        spawner,
        "WexBot",
        Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0x0f]),
    )
    .await;

    let mut led_levels = [Level::Low; 4];
    // ------------------- START MODIFYING HERE -----------------------
    match COMMAND_SIGNAL.wait().await {
        b'w' => robot.set_speeds(50, 50),
        b's' => robot.set_speeds(-50, -50),
        b'a' => robot.set_speeds(0, 50),
        b'd' => robot.set_speeds(50, 0),
        led @ b'1'..=b'4' => {
            let idx = (led - 49) as usize;
            let level_bool: bool = led_levels[idx].into();
            let flipped_level: Level = (!level_bool).into();
            led_levels[idx] = flipped_level;

            match led {
                b'1' => robot.set_led_3(flipped_level),
                b'2' => robot.set_led_2(flipped_level),
                b'3' => robot.set_led_1(flipped_level),
                b'4' => robot.set_led_0(flipped_level),
                _ => unreachable!(),
            }
        }
        KILL | _ => robot.halt_motors(),
    }
    // ----------------------------------------------------------------
}
