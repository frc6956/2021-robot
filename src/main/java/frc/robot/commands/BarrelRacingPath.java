// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.trajectory.*;
import frc.robot.subsystems.Drivetrain;

public class BarrelRacingPath extends PathCommand {
  public BarrelRacingPath(Drivetrain drivetrain) {
    super(drivetrain, TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      List.of(
        fieldPointsToMeters(1, 3, 0),
        fieldPointsToMeters(5, 3, 0),
        fieldPointsToMeters(6, 2, 270),
        fieldPointsToMeters(5, 1, 180),
        fieldPointsToMeters(4, 2, 90),
        fieldPointsToMeters(5, 3, 0),
        fieldPointsToMeters(8, 3, 0),
        fieldPointsToMeters(9, 4, 90),
        fieldPointsToMeters(8, 5, 180),
        fieldPointsToMeters(7, 4, 270),
        fieldPointsToMeters(7, 3, 315),
        fieldPointsToMeters(8, 2, 315),
        fieldPointsToMeters(10, 1, 0),
        fieldPointsToMeters(11, 2, 90),
        fieldPointsToMeters(10, 3, 180),
        fieldPointsToMeters(1, 3, 180)
      ),
      forwardConfig));
      
    drivetrain.resetOdometry(getInitialPose());
  }
}
