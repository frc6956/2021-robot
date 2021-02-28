// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ColorSpinner;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //Subsystems
  private final PowerDistributionPanel m_pdp = new PowerDistributionPanel();
  private final Gyro m_gyro = new PigeonGyro();
  private final Drivetrain m_drivetrain = new Drivetrain(m_gyro);
  private final Intake m_intake = new Intake();
  private final Conveyor m_conveyor = new Conveyor(m_pdp);
  private final Feeder m_feeder = new Feeder();
  private final LinearSlide m_slide = new LinearSlide();
  private final ColorSpinner m_spinner = new ColorSpinner();
  private final Shooter m_shooter = new Shooter();

  //OI Devices
  private final Joystick m_driverLeftJoystick = new Joystick(Constants.USB.driverJoyLeft);
  private final Joystick m_driverRightJoystick = new Joystick(Constants.USB.driverJoyRight);
  private final XboxController m_operatorController = new XboxController(Constants.USB.operatorController);

  //Commands
    //Driver
  private final Command m_splitArcadeJoystick = new RunCommand(
    () -> m_drivetrain.arcadeDrive(-m_driverRightJoystick.getY(), m_driverLeftJoystick.getX()), m_drivetrain);

  private final Command m_tankJoystick = new RunCommand(
    () -> m_drivetrain.tankDrive(-m_driverLeftJoystick.getY(), -m_driverRightJoystick.getY()), m_drivetrain);

  private final Command m_driverSwitchHigh = new InstantCommand(
    () -> m_drivetrain.highGear());
  
  private final Command m_driverSwitchLow = new InstantCommand(
    () -> m_drivetrain.lowGear());

  private final Command m_invertDrive = new InstantCommand(
    () -> m_drivetrain.reverse(!m_drivetrain.isReversed()));

     //Operator
  private final Command m_TeleopIntake  = new RunCommand(
    () -> m_intake.setIntakeSpeed(-m_operatorController.getY(Hand.kLeft)), m_intake);

  private final Command m_TeleopConveyor = new RunCommand(
    () -> m_conveyor.setConveyorSpeed(m_operatorController.getY(Hand.kLeft)), m_conveyor);

  private final Command m_TeleopFeeder = new RunCommand(
    () -> m_feeder.setFeedSpeed(m_operatorController.getY(Hand.kRight)), m_feeder);

  private final Command m_TeleopShooter = new RunCommand(
    () -> m_shooter.setShooterRPM(m_operatorController.getTriggerAxis(Hand.kRight) * Constants.Shooter.kMaxRPM), m_shooter);

  private final Command m_ShooterGreen = new RunCommand(
    () -> m_shooter.setShooterRPM(1750), m_shooter);
  private final Command m_ShooterYellow = new RunCommand(
    () -> m_shooter.setShooterRPM(2000), m_shooter);
  private final Command m_ShooterBlue = new RunCommand(
    () -> m_shooter.setShooterRPM(3000), m_shooter);
  private final Command m_ShooterRed = new RunCommand(
    () -> m_shooter.setShooterRPM(4000), m_shooter);

  private final Command m_TeleopSlideIn = new RunCommand(
    () -> m_slide.actuateIn(), m_slide);

  private final Command m_TeleopSlideOut = new RunCommand(
    () -> m_slide.actuateOut(), m_slide);

  private final Command m_SpinnerUp = new RunCommand(
    () -> m_spinner.up(), m_spinner);

  private final Command m_SpinnerDown = new RunCommand(
    () -> m_spinner.down(), m_spinner);

  private final Command m_SpinnerSpinR = new RunCommand(
    () -> m_spinner.setRPM(640), m_spinner);

  private final Command m_SpinnerSpinL = new RunCommand(
    () -> m_spinner.setRPM(-640), m_spinner);
  
  private final Command m_SpinnerStop = new RunCommand(
    () -> m_spinner.setWheelSpeed(0), m_spinner);

  private final SequentialCommandGroup m_AutoGrab = new SequentialCommandGroup(new Slide(m_slide),  new WaitCommand(2), new Slide(m_slide) );

  //Auton
  private final Command autonShoot = new RunCommand(
    () -> m_shooter.setShooterSpeed(1), m_shooter);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Default Commands
    //m_drivetrain.setDefaultCommand(m_splitArcadeJoystick);
    m_drivetrain.setDefaultCommand(m_tankJoystick);
    m_intake.setDefaultCommand(m_TeleopIntake);
    m_conveyor.setDefaultCommand(m_TeleopConveyor);
    m_feeder.setDefaultCommand(m_TeleopFeeder);
    m_shooter.setDefaultCommand(m_TeleopShooter);

    CameraServer.getInstance().startAutomaticCapture(0);
    //CameraServer.getInstance().startAutomaticCapture(1);
    SmartDashboard.putData(m_pdp);
    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    //Driver Configs
        //new JoystickButton(m_driverLeftJoystick, 5).whenPressed(m_driverSwitchHigh);
        //new JoystickButton(m_driverRightJoystick, 6).whenPressed(m_driverSwitchLow);
        new JoystickButton(m_driverRightJoystick, 6).whenPressed(m_driverSwitchHigh);
        
        new JoystickButton(m_driverRightJoystick, 6).whenReleased(m_driverSwitchLow);
        new JoystickButton(m_driverRightJoystick, 1).whenPressed(m_invertDrive);
        new JoystickButton(m_driverLeftJoystick, 1).whenPressed(m_TeleopSlideIn);
    
    //Operator Configs
        new JoystickButton(m_operatorController, XboxController.Button.kY.value).whenPressed(m_SpinnerUp);
        new JoystickButton(m_operatorController, XboxController.Button.kX.value).whenPressed(m_SpinnerDown);
        new JoystickButton(m_operatorController, XboxController.Button.kA.value).whenPressed(m_TeleopSlideIn);
        new JoystickButton(m_operatorController, XboxController.Button.kB.value).whenPressed(m_TeleopSlideOut);
    
        new JoystickButton(m_operatorController, XboxController.Button.kBumperLeft.value).whileHeld(m_SpinnerSpinL);
        new JoystickButton(m_operatorController, XboxController.Button.kBumperLeft.value).whenReleased(m_SpinnerStop);
        new JoystickButton(m_operatorController, XboxController.Button.kBumperRight.value).whileHeld(m_SpinnerSpinR);
        new JoystickButton(m_operatorController, XboxController.Button.kBumperRight.value).whenReleased(m_SpinnerStop);
    
        new JoystickButton(m_driverRightJoystick, 7).whileHeld(m_ShooterGreen);
        new JoystickButton(m_driverRightJoystick, 8).whileHeld(m_ShooterYellow);
        new JoystickButton(m_driverRightJoystick, 9).whileHeld(m_ShooterBlue);
        new JoystickButton(m_driverRightJoystick, 10).whileHeld(m_ShooterRed);
  }

  private Pose2d fieldPointsToMeters(double xFieldPoints, double yFieldPoints, double rotationDegrees) {
      double xMeters = xFieldPoints * 30 * 0.0254;
      double yMeters = yFieldPoints * 30 * 0.0254;
      double rotationRadians = rotationDegrees * Math.PI / 180;
      return new Pose2d(xMeters, yMeters, new Rotation2d(rotationRadians));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, 0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 1, new Rotation2d(0)),
            // Pass config
            config);

    Trajectory poseTraj = 
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        List.of( new Pose2d(0, 1.5, new Rotation2d(0)), 
        new Pose2d(2, 1.5, new Rotation2d(0)),
        new Pose2d(2.5, 2, new Rotation2d(1.57)),
        new Pose2d(2, 2.5, new Rotation2d(1.57*2)),
        new Pose2d(1.5, 2, new Rotation2d(1.57*3)),
        new Pose2d(1.5, 0, new Rotation2d(1.57*3))),
        config);

    Trajectory slalomWaypoints = 
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        List.of(
        fieldPointsToMeters(1.5, 1, 0),
        fieldPointsToMeters(2, 1, 0),
        fieldPointsToMeters(3, 2, 45),
        fieldPointsToMeters(4, 3, 0),
        fieldPointsToMeters(8, 3, 0),
        fieldPointsToMeters(9, 2, 315),
        fieldPointsToMeters(10, 1, 0),
        fieldPointsToMeters(11, 2, 90),
        fieldPointsToMeters(10, 3, 180),
        fieldPointsToMeters(9, 2, 225),
        fieldPointsToMeters(8, 1, 180),
        fieldPointsToMeters(4, 1, 180),
        fieldPointsToMeters(3, 2, 135),
        fieldPointsToMeters(2, 3, 180),
        fieldPointsToMeters(1, 3, 180)),
        config);
    
    String trajectoryJSON = "paths/SlalomPath.wpilib.json";
    Trajectory slalom = new Trajectory();
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        slalom = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unstable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    Trajectory traj = slalomWaypoints;

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            traj,
            m_drivetrain::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            m_drivetrain::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_drivetrain::tankDriveVolts,
            m_drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    m_drivetrain.resetOdometry(traj.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_drivetrain.tankDriveVolts(0, 0));
  }
}
