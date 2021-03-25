// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.trajectory.*;
import frc.robot.subsystems.Drivetrain;

public class GalacticSearchCombined extends PathCommand {
  public GalacticSearchCombined(Drivetrain drivetrain) {
    super(drivetrain, TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      List.of(
        fieldPointsToMeters(1, 5, 0),
        fieldPointsToMeters(2, 5, 0),
        fieldPointsToMeters(3, 4, 270),
        fieldPointsToMeters(3, 3, 270),
        fieldPointsToMeters(5, 2, 315),
        fieldPointsToMeters(6, 1, 0),
        fieldPointsToMeters(6.5, 1.5, 90),
        fieldPointsToMeters(6, 2, 135),
        fieldPointsToMeters(5, 4, 90),
        fieldPointsToMeters(6, 5, 0),
        fieldPointsToMeters(6.5, 4.5, 270),
        fieldPointsToMeters(7, 4, 0),
        fieldPointsToMeters(8, 4, 0),
        fieldPointsToMeters(9, 3, 315),
        fieldPointsToMeters(10, 2, 0),
        fieldPointsToMeters(11, 2, 0)
      ),
      forwardConfig));

    drivetrain.resetOdometry(getInitialPose());
  }
}
