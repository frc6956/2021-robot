// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj.trajectory.constraint.*;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.Drivetrain;

public class PathCommand extends RamseteCommand {
  private Trajectory m_path;
  
  // Create a voltage constraint to ensure we don't accelerate too fast
  private static TrajectoryConstraint autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
        DriveConstants.ksVolts,
        DriveConstants.kvVoltSecondsPerMeter,
        DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      10);

  // Create config for trajectory
  protected static TrajectoryConfig forwardConfig =
    new TrajectoryConfig(
          AutoConstants.kMaxSpeedMetersPerSecond,
          AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(DriveConstants.kDriveKinematics)
      // Apply the voltage constraint
      .addConstraint(autoVoltageConstraint)
      .setReversed(false);

  // Create config for trajectory
  protected static TrajectoryConfig reverseConfig =
    new TrajectoryConfig(
          AutoConstants.kMaxSpeedMetersPerSecond,
          AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(DriveConstants.kDriveKinematics)
      // Apply the voltage constraint
      .addConstraint(autoVoltageConstraint)
      .setReversed(true);

  /** Creates a new PathCommandBase. */
  public PathCommand(Drivetrain drivetrain, Trajectory path) {
    super(
      path,
      drivetrain::getPose,
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      new SimpleMotorFeedforward(
          DriveConstants.ksVolts,
          DriveConstants.kvVoltSecondsPerMeter,
          DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      drivetrain::getWheelSpeeds,
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      drivetrain::tankDriveVolts,
      drivetrain);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_path = path;
  }

  public static Pose2d fieldPointsToMeters(double xFieldPoints, double yFieldPoints, double rotationDegrees) {
    double xMeters = xFieldPoints * 30 * 0.0254;
    double yMeters = yFieldPoints * 30 * 0.0254;
    return new Pose2d(xMeters, yMeters, Rotation2d.fromDegrees(rotationDegrees));
  }

  public Pose2d getInitialPose() {
    return m_path.getInitialPose();
  }
}
