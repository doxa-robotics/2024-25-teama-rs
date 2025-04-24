use core::{
    f64::consts::{FRAC_PI_2, FRAC_PI_4, FRAC_PI_8, PI},
    time::Duration,
};

use vexide::{prelude::Motor, task::spawn, time::sleep};

use crate::{
    subsystems::drivetrain_actions::{self, CONFIG, TILES_TO_MM},
    Robot,
};

async fn route(robot: &mut Robot) {
    robot
        .tracking
        .borrow_mut()
        .set_pose((360.0, -1400.0, 0.74 - PI).into());

    // Alliance score
    robot
        .drivetrain
        .action(drivetrain_actions::forward(0.23, CONFIG))
        .await;
    robot
        .lady_brown
        .set_state(crate::subsystems::lady_brown::LadyBrownState::MaxExpansion);
    sleep(Duration::from_millis(700)).await;
    robot
        .lady_brown
        .set_state(crate::subsystems::lady_brown::LadyBrownState::Initial);
    // robot
    //     .drivetrain
    //     .action(drivetrain_actions::turn_to_point(
    //         // (2.0, -1.0).into(),
    //         (-1.0, -4.0).into(),
    //         CONFIG.with_turn_error_tolerance(0.3),
    //     ))
    //     .await;

    // // Corner
    // let intake_clone = robot.intake.clone();
    // spawn(async move {
    //     sleep(Duration::from_millis(1500)).await;
    //     intake_clone.partial_intake();
    // })
    // .detach();
    // robot
    //     .drivetrain
    //     .action(drivetrain_actions::smooth_to_point(
    //         (2.2, -2.2, PI * 0.75).into(),
    //         2.0,
    //         4.0,
    //         CONFIG
    //             .with_linear_error_tolerance(100.0)
    //             .with_linear_limit(Motor::V5_MAX_VOLTAGE * 0.7),
    //         false,
    //     ))
    //     .await;
    // robot
    //     .drivetrain
    //     .action(drivetrain_actions::turn_to_point(
    //         (3.0, -3.0).into(),
    //         CONFIG.with_turn_error_tolerance(0.15),
    //     ))
    //     .await;
    // robot
    //     .drivetrain
    //     .action(drivetrain_actions::forward(
    //         0.1,
    //         CONFIG
    //             .with_linear_error_tolerance(300.0)
    //             .with_linear_limit(Motor::V5_MAX_VOLTAGE * 0.3),
    //     ))
    //     .await;

    // Get goal
    // robot
    //     .drivetrain
    //     .action(drivetrain_actions::smooth_to_point(
    //         (1.0, -2.0, FRAC_PI_2).into(),
    //         1.0,
    //         3.0,
    //         true,
    //         true,
    //         CONFIG
    //             .with_linear_limit(Motor::V5_MAX_VOLTAGE * 0.4)
    //             .with_pursuit_lookahead(80.0)
    //             .with_pursuit_turn_kp(4.0)
    //             .with_linear_error_tolerance(120.0),
    //     ))
    //     .await;
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (1.2, -2.0).into(),
            true,
            CONFIG, // CONFIG.with_linear_limit(Motor::V5_MAX_VOLTAGE * 0.6),
        ))
        .await;
    let mut clamp_clone = robot.clamp.clone();
    spawn(async move {
        sleep(Duration::from_millis(1400)).await;
        clamp_clone.extend();
    })
    .detach();
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (1.2, -1.0).into(),
            true,
            CONFIG, // CONFIG.with_linear_limit(Motor::V5_MAX_VOLTAGE * 0.6),
        ))
        .await;
    robot.clamp.extend();
    // sleep(Duration::from_millis(500)).await;
    robot.intake.run(vexide::prelude::Direction::Forward);

    // Get ring at (2.0, -1.0)
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (2.0, -1.0).into(),
            false,
            CONFIG.with_linear_error_tolerance(100.0),
        ))
        .await;
    sleep(Duration::from_millis(200)).await;

    // Get the ring in front of alliance stake
    robot.intake_raiser.extend();
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (0.0, -2.0).into(),
            false,
            CONFIG,
        ))
        .await;
    robot.intake_raiser.retract();

    // Drive to bar touch
    robot
        .lady_brown
        .set_state(crate::subsystems::lady_brown::LadyBrownState::MaxExpansion);
    robot
        .drivetrain
        .action(drivetrain_actions::turn_to_point(
            (0.0, 0.0).into(),
            CONFIG.with_turn_error_tolerance(0.2),
        ))
        .await;
    robot
        .drivetrain
        .action(drivetrain_actions::forward(0.8, CONFIG))
        .await;

    // TODO: touch
}

pub async fn blue(robot: &mut Robot) {
    robot.tracking.borrow_mut().set_reverse(true);
    robot
        .intake
        .set_accept(Some(crate::subsystems::intake::RingColor::Blue));
    route(robot).await;
}

pub async fn red(robot: &mut Robot) {
    robot.tracking.borrow_mut().set_reverse(false);
    robot
        .intake
        .set_accept(Some(crate::subsystems::intake::RingColor::Red));
    route(robot).await;
}
