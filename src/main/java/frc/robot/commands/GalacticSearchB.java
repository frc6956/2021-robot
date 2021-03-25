// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.trajectory.*;
import frc.robot.subsystems.Drivetrain;

public class GalacticSearchB extends PathCommand {
  public GalacticSearchB(Drivetrain drivetrain) {
    super(drivetrain, TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      List.of(
        fieldPointsToMeters(1, 4, 0),
        fieldPointsToMeters(3, 4, 0),
        fieldPointsToMeters(5, 2, 0),
        fieldPointsToMeters(6, 2, 0),
        fieldPointsToMeters(7, 4, 90),
        fieldPointsToMeters(8, 4, 0),
        fieldPointsToMeters(10, 2, 0),
        fieldPointsToMeters(11, 2, 0)
      ),
      forwardConfig));

    drivetrain.resetOdometry(getInitialPose());
  }
}
