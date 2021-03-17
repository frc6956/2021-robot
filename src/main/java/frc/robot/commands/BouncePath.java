// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.trajectory.*;
import frc.robot.subsystems.Drivetrain;

public class BouncePath extends SequentialCommandGroup {
  /** Creates a new BouncePath. */
  public BouncePath(Drivetrain drivetrain) {
    PathCommand path1 = new PathCommand(drivetrain, TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      List.of(
        PathCommand.fieldPointsToMeters(1, 3, 0),
        PathCommand.fieldPointsToMeters(2, 3, 0),
        PathCommand.fieldPointsToMeters(3, 4, 90),
        PathCommand.fieldPointsToMeters(3, 5, 90)
      ),
      PathCommand.forwardConfig));

    PathCommand path2 = new PathCommand(drivetrain, TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      List.of(
        PathCommand.fieldPointsToMeters(3, 4, 90),
        PathCommand.fieldPointsToMeters(4, 2, 90),
        PathCommand.fieldPointsToMeters(5, 1, 180),
        PathCommand.fieldPointsToMeters(6, 2, 270),
        PathCommand.fieldPointsToMeters(6, 5, 270)
      ),
      PathCommand.reverseConfig));

    PathCommand path3 = new PathCommand(drivetrain, TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      List.of(
        PathCommand.fieldPointsToMeters(6, 5, 270),
        PathCommand.fieldPointsToMeters(6, 2, 270),
        PathCommand.fieldPointsToMeters(7, 1, 0),
        PathCommand.fieldPointsToMeters(8, 1, 0),
        PathCommand.fieldPointsToMeters(9, 2, 90),
        PathCommand.fieldPointsToMeters(9, 5, 90)
      ),
      PathCommand.forwardConfig));

    PathCommand path4 = new PathCommand(drivetrain, TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      List.of(
        PathCommand.fieldPointsToMeters(9, 5, 90),
        PathCommand.fieldPointsToMeters(9, 4, 90),
        PathCommand.fieldPointsToMeters(10, 3.5, 180),
        PathCommand.fieldPointsToMeters(11, 3.5, 180)
      ),
      PathCommand.reverseConfig));

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(path1, path2, path3, path4);

    drivetrain.resetOdometry(path1.getInitialPose());
  }
}
