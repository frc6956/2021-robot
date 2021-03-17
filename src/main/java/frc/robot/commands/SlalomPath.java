// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.trajectory.*;
import frc.robot.subsystems.Drivetrain;

public class SlalomPath extends PathCommand {
  public SlalomPath(Drivetrain drivetrain) {
    super(drivetrain, TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      List.of(
        fieldPointsToMeters(1.5, 1, 0),
        fieldPointsToMeters(2.2, 1, 0),
        fieldPointsToMeters(3, 2, 90),
        fieldPointsToMeters(4, 3, 0),
        fieldPointsToMeters(8, 3, 0),
        fieldPointsToMeters(8.8, 2, 270),
        fieldPointsToMeters(10, 1, 0),
        fieldPointsToMeters(11, 2, 90),
        fieldPointsToMeters(10, 3, 180),
        fieldPointsToMeters(9.2, 2, 270),
        fieldPointsToMeters(8, 1, 180),
        fieldPointsToMeters(4, 1, 180),
        fieldPointsToMeters(3, 2, 90),
        fieldPointsToMeters(2, 3, 180),
        fieldPointsToMeters(1, 3, 180)
      ),
      forwardConfig));

    drivetrain.resetOdometry(getInitialPose());
  }
}
