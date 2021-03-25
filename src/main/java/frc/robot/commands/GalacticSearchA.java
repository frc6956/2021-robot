// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.trajectory.*;
import frc.robot.subsystems.Drivetrain;

public class GalacticSearchA extends PathCommand {
  public GalacticSearchA(Drivetrain drivetrain) {
    super(drivetrain, TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      List.of(
        fieldPointsToMeters(1,3,0),
        fieldPointsToMeters(3, 3, 0),
        fieldPointsToMeters(5, 2, 270),
        fieldPointsToMeters(6, 1, 0),
        fieldPointsToMeters(7, 2, 90),
        fieldPointsToMeters(6, 3, 135),
        fieldPointsToMeters(5, 4, 90),
        fieldPointsToMeters(6, 5, 0),
        fieldPointsToMeters(7, 4, 270),
        fieldPointsToMeters(9, 3, 0),
        fieldPointsToMeters(11, 3, 0)
      ),
      forwardConfig));

    drivetrain.resetOdometry(getInitialPose());
  }
}
