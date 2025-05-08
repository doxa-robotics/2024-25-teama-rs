use core::{f64::consts::FRAC_PI_2, time::Duration};

use vexide::{prelude::Direction, time::sleep};

use crate::{
    subsystems::drivetrain_actions::{self, CONFIG, TILES_TO_MM},
    Robot,
};

async fn route(robot: &mut Robot) {
    // Starting position
    robot
        .tracking
        .borrow_mut()
        .set_pose((-600.0 * 2.0 - 350.0, -600.0 * 2.0 - 120.0, FRAC_PI_2).into());

    //doinker

    robot.doinker.non_dominant().extend();
    robot.intake.partial_intake();
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (-2.7, -0.5).into(),
            false,
            CONFIG
                .with_linear_error_tolerance(200.0)
                .with_linear_velocity_tolerance(400.0)
                .with_linear_tolerance_duration(Duration::ZERO)
                .with_turn_error_tolerance(0.4)
                .with_turn_velocity_tolerance(200.0),
        ))
        .await;
    robot.doinker.non_dominant().retract();
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (-1.6, -0.4).into(),
            false,
            CONFIG,
        ))
        .await;
    let mut clamp = robot.clamp.clone();
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (-0.9, -1.1).into(),
            true,
            CONFIG,
        ))
        .with_callback(move |pose| {
            if pose.x() > -1.3 * TILES_TO_MM {
                clamp.extend();
            }
        })
        .await;
    robot.clamp.extend();
    robot.intake.run(Direction::Forward);
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (-2.0, -2.4).into(),
            false,
            CONFIG,
        ))
        .await;
    sleep(Duration::from_millis(600)).await;
    robot.intake_raiser.extend();
    robot
        .drivetrain
        .action(drivetrain_actions::drive_to_point(
            (-2.7, -2.7).into(),
            false,
            CONFIG,
        ))
        .await;
    robot.intake_raiser.retract();
}

pub async fn blue(robot: &mut Robot) {
    robot.tracking.borrow_mut().set_reverse(true);
    robot
        .intake
        .set_accept(Some(crate::subsystems::intake::RingColor::Blue));
    robot
        .doinker
        .set_mirrored_state(libdoxa::subsystems::pneumatic::MirroredState::Mirrored);
}

pub async fn red(robot: &mut Robot) {
    robot.tracking.borrow_mut().set_reverse(false);
    robot
        .intake
        .set_accept(Some(crate::subsystems::intake::RingColor::Red));
    route(robot).await;
}
