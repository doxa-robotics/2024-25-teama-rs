use core::{f64::consts::FRAC_PI_2, time::Duration};

use vexide::time::sleep;

use crate::{
    subsystems::drivetrain_actions::{self, CONFIG, TILES_TO_MM},
    Robot,
};

async fn route(robot: &mut Robot) {
    robot
        .tracking
        .borrow_mut()
        .set_pose((1.0 * TILES_TO_MM, -2.5 * TILES_TO_MM, -FRAC_PI_2).into());

    // stake
    let mut clamp = robot.clamp.clone();
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (1.0, -0.8).into(),
            true,
            CONFIG
                .with_linear_error_tolerance(200.0)
                .with_linear_velocity_tolerance(600.0),
        ))
        .with_callback(move |pose| {
            if pose.y() > -1.1 * TILES_TO_MM {
                clamp.extend();
            }
        })
        .await;
    robot.intake.run(vexide::prelude::Direction::Forward);
    sleep(Duration::from_millis(200)).await;

    // Get ring at (2.0, -1.0)
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (2.1, -1.0).into(),
            false,
            CONFIG
                .with_linear_error_tolerance(200.0)
                .with_linear_velocity_tolerance(600.0),
        ))
        .await;
    sleep(Duration::from_millis(200)).await;

    // Corner
    let intake = robot.intake.clone();
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (2.7, -2.7).into(),
            false,
            CONFIG.with_linear_error_tolerance(100.0),
        ))
        .with_callback(move |pose| {
            if pose.y() < -2.0 * TILES_TO_MM {
                intake.run(vexide::prelude::Direction::Reverse);
            }
        })
        .await;
    robot.intake.run(vexide::prelude::Direction::Forward);
    sleep(Duration::from_millis(400)).await;
    robot
        .drivetrain
        .action(drivetrain_actions::forward(
            -0.3,
            CONFIG
                .with_linear_error_tolerance(100.0)
                .with_linear_velocity_tolerance(200.0)
                .with_linear_tolerance_duration(Duration::ZERO),
        ))
        .await;
    robot
        .drivetrain
        .action(drivetrain_actions::forward(
            0.3,
            CONFIG.with_linear_error_tolerance(100.0),
        ))
        .await;
    sleep(Duration::from_millis(300)).await;
    robot
        .drivetrain
        .action(drivetrain_actions::forward(
            -0.3,
            CONFIG
                .with_linear_error_tolerance(100.0)
                .with_linear_velocity_tolerance(300.0),
        ))
        .await;
    // Since we have time, an extra pass
    robot
        .drivetrain
        .action(drivetrain_actions::forward(
            0.3,
            CONFIG.with_linear_error_tolerance(100.0),
        ))
        .await;
    robot
        .drivetrain
        .action(drivetrain_actions::forward(
            -0.5,
            CONFIG
                .with_linear_error_tolerance(300.0)
                .with_linear_velocity_tolerance(500.0),
        ))
        .await;

    // Drive to middle
    robot.intake.stop(); // to prevent flying ring
    robot
        .lady_brown
        .set_state(crate::subsystems::lady_brown::LadyBrownState::MaxExpansion);
    let intake = robot.intake.clone();
    let mut flag = false;
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (1.0, -0.7).into(),
            false,
            CONFIG,
        ))
        .with_callback(move |pose| {
            if pose.x() < -1.5 * TILES_TO_MM && !flag {
                flag = true;
                intake.run(vexide::prelude::Direction::Forward);
            }
        })
        .await;
    robot
        .drivetrain
        .action(drivetrain_actions::turn_to_point(
            (-2.0, 1.5).into(),
            CONFIG,
        ))
        .await;
}

pub async fn blue_positive(robot: &mut Robot) {
    robot.tracking.borrow_mut().set_reverse(true);
    robot
        .intake
        .set_accept(Some(crate::subsystems::intake::RingColor::Blue));
    route(robot).await;
}

pub async fn red_positive(robot: &mut Robot) {
    robot.tracking.borrow_mut().set_reverse(false);
    robot
        .intake
        .set_accept(Some(crate::subsystems::intake::RingColor::Red));
    route(robot).await;
}

pub async fn blue_negative(robot: &mut Robot) {
    robot.tracking.borrow_mut().set_reverse(false);
    robot
        .intake
        .set_accept(Some(crate::subsystems::intake::RingColor::Blue));
    route(robot).await;
}

pub async fn red_negative(robot: &mut Robot) {
    robot.tracking.borrow_mut().set_reverse(true);
    robot
        .intake
        .set_accept(Some(crate::subsystems::intake::RingColor::Red));
    route(robot).await;
}
