// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  private double zeroDistance = 0;
  private final DifferentialDrive m_drive;

  private final WPI_TalonSRX m_leftSRX;
  private final WPI_TalonSRX m_rightSRX;

  private final WPI_VictorSPX m_leftSPX1;
  private final WPI_VictorSPX m_rightSPX1;

  private final WPI_VictorSPX m_leftSPX2;
  private final WPI_VictorSPX m_rightSPX2;

  private GyroSubsystem m_gyro = new GyroSubsystem();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_leftSRX = new WPI_TalonSRX(Constants.CAN.leftDriveMotor);
    m_leftSPX1 = new WPI_VictorSPX(Constants.CAN.leftDriveSPX1);
    m_leftSPX2 = new WPI_VictorSPX(Constants.CAN.leftDriveSPX2);
    
    m_rightSRX = new WPI_TalonSRX(Constants.CAN.rightDriveMotor);
    m_rightSPX1 = new WPI_VictorSPX(Constants.CAN.rightDriveSPX1);
    m_rightSPX2 = new WPI_VictorSPX(Constants.CAN.rightDriveSPX2);

    m_drive = new DifferentialDrive(m_leftSRX, m_rightSRX);

    m_leftSPX1.follow(m_leftSRX);
    m_leftSPX2.follow(m_leftSRX);
    m_rightSPX1.follow(m_rightSRX);
    m_rightSPX2.follow(m_rightSRX);

    m_leftSRX.setInverted(InvertType.InvertMotorOutput);
    m_leftSPX1.setInverted(InvertType.FollowMaster);
    m_leftSPX2.setInverted(InvertType.FollowMaster);
    m_rightSRX.setInverted(InvertType.None);
    m_rightSPX1.setInverted(InvertType.FollowMaster);
    m_rightSPX2.setInverted(InvertType.FollowMaster);

    m_drive.setRightSideInverted(false);

    m_leftSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
   // m_leftSRX.setSensorPhase(true);
    m_rightSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    // Sets the distance per pulse for the encoders
    /*m_leftSRX.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);*/

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(),
        m_leftSRX.getSelectedSensorPosition() / Constants.DriveConstants.ticksPerMeter,
        m_rightSRX.getSelectedSensorPosition() / Constants.DriveConstants.ticksPerMeter);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        m_leftSRX.getSelectedSensorVelocity() / Constants.DriveConstants.ticksPerMeter,
        m_rightSRX.getSelectedSensorVelocity() / Constants.DriveConstants.ticksPerMeter);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Drives the robot using tank controls.
   *
   * @param left  the commanded left movement
   * @param right the commanded right movement
   */
  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftSRX.setVoltage(leftVolts);
    m_rightSRX.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftSRX.setSelectedSensorPosition(0);
    m_rightSRX.setSelectedSensorPosition(0);
    resetDistanceTravelled();
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  public double getDistanceTravelled() {
    return getRawDistanceTravelled() - zeroDistance;
  }

  /**
   * Resets the distance travelled
   */
  public void resetDistanceTravelled() {
    zeroDistance = getRawDistanceTravelled();
  }
  
  protected double getRawDistanceTravelled() {
    double total = m_leftSRX.getSelectedSensorPosition(0) / Constants.DriveConstants.ticksPerMeter;
    total += m_rightSRX.getSelectedSensorPosition(0) / Constants.DriveConstants.ticksPerMeter;
    return (total / 2);
  }
}
