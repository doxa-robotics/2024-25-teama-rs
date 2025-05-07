use core::{f64::consts::FRAC_PI_2, time::Duration};

use vexide::time::sleep;

use crate::{
    subsystems::drivetrain_actions::{self, CONFIG, TILES_TO_MM},
    Robot,
};

async fn route(robot: &mut Robot) {
    sleep(Duration::from_millis(1000)).await;
    // Starting position
    robot
        .tracking
        .borrow_mut()
        .set_pose((600.0 * 2.0, -600.0 * 2.5, FRAC_PI_2).into());

    robot.intake.run(vexide::prelude::Direction::Forward);
    robot.doinker.non_dominant().extend();
    // robot.intake_raiser.extend();

    let mut intake_raiser = robot.intake_raiser.clone();
    let intake = robot.intake.clone();
    let mut flag_1 = false;
    let mut flag_2 = false;
    let mut flag_3 = false;
    let mut flag_4 = false;
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (2.0, -0.4).into(),
            false,
            CONFIG
                .with_turn_tolerance_duration(core::time::Duration::ZERO)
                .with_turn_error_tolerance(1.0)
                .with_turn_velocity_tolerance(200.0)
                .with_boomerang_lock_distance(2.0 * TILES_TO_MM)
                .with_linear_limit(400.0),
        ))
        .with_callback(move |pose| {
            if pose.y() > -1.0 * crate::subsystems::drivetrain_actions::TILES_TO_MM && !flag_1 {
                flag_1 = true;
                intake_raiser.retract();
            }
            if pose.y() > -2.1 * crate::subsystems::drivetrain_actions::TILES_TO_MM && !flag_2 {
                flag_2 = true;
                // intake_raiser.extend();
            }
            if pose.y() > -0.6 * crate::subsystems::drivetrain_actions::TILES_TO_MM && !flag_3 {
                flag_3 = true;
                intake.partial_intake();
            }
            if pose.y() > 2.47 * crate::subsystems::drivetrain_actions::TILES_TO_MM && !flag_4 {
                flag_4 = true;
                intake.stop();
            }
        })
        .await;

    robot
        .drivetrain
        .action(drivetrain_actions::turn_to_point(
            (3.0, -1.0).into(),
            CONFIG.with_turn_error_tolerance(0.05),
        ))
        .await;

    robot.doinker.non_dominant().retract();

    let mut clamp = robot.clamp.clone();
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (0.7, -1.2).into(),
            true,
            CONFIG
                .with_linear_error_tolerance(100.0)
                .with_linear_velocity_tolerance(300.0)
                .with_linear_tolerance_duration(Duration::ZERO),
        ))
        .with_callback(move |pose| {
            if pose.x() < 1.1 * crate::subsystems::drivetrain_actions::TILES_TO_MM {
                clamp.extend();
            }
        })
        .await;
    let old_accept = robot.intake.accept();
    robot.intake.set_accept(None);
    robot.intake.run(vexide::prelude::Direction::Forward);
    sleep(Duration::from_millis(200)).await;
    robot.intake.set_accept(old_accept);
    robot.intake.run(vexide::prelude::Direction::Forward);
    sleep(Duration::from_millis(800)).await;
    robot.clamp.retract();
    robot.intake.stop();

    // let mut clamp = robot.clamp.clone();
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (2.4, -0.25).into(),
            true,
            CONFIG
                .with_linear_error_tolerance(100.0)
                .with_turn_error_tolerance(0.15),
        ))
        .with_callback(move |pose| {
            if pose.x() > 2.4 * crate::subsystems::drivetrain_actions::TILES_TO_MM {
                // clamp.extend();
            }
        })
        .await;
    robot.clamp.extend();

    // Preload
    robot.intake.run(vexide::prelude::Direction::Forward);
    robot
        .drivetrain
        .action(drivetrain_actions::forward(
            0.5,
            CONFIG
                .with_linear_error_tolerance(200.0)
                .with_linear_velocity_tolerance(300.0)
                .with_linear_tolerance_duration(Duration::ZERO),
        ))
        .await;
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (2.35, -2.3).into(),
            false,
            CONFIG
                .with_turn_error_tolerance(0.2)
                .with_turn_tolerance_duration(Duration::ZERO)
                .with_turn_velocity_tolerance(300.0),
        ))
        .await;

    // Corner ring
    sleep(Duration::from_millis(1000)).await;
    let intake = robot.intake.clone();
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (2.8, -2.8).into(),
            false,
            CONFIG
                .with_linear_error_tolerance(200.0)
                .with_linear_velocity_tolerance(100.0)
                .with_linear_tolerance_duration(Duration::ZERO),
        ))
        .with_callback(move |pose| {
            if pose.x() > 2.6 {
                intake.run(vexide::prelude::Direction::Reverse);
            }
        })
        .await;
    robot.intake.run(vexide::prelude::Direction::Forward);
    sleep(Duration::from_millis(500)).await;
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
        .action(drivetrain_actions::forward(0.3, CONFIG))
        .await;
    sleep(Duration::from_millis(1000)).await;
}

pub async fn blue(robot: &mut Robot) {
    robot.tracking.borrow_mut().set_reverse(true);
    robot
        .intake
        .set_accept(Some(crate::subsystems::intake::RingColor::Blue));
    robot
        .doinker
        .set_mirrored_state(libdoxa::subsystems::pneumatic::MirroredState::Mirrored);
    route(robot).await;
}

pub async fn red(robot: &mut Robot) {
    robot.tracking.borrow_mut().set_reverse(false);
    robot
        .intake
        .set_accept(Some(crate::subsystems::intake::RingColor::Red));
    robot
        .doinker
        .set_mirrored_state(libdoxa::subsystems::pneumatic::MirroredState::Normal);
    route(robot).await;
}
