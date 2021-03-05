// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private double zeroDistance = 0;
  private final DifferentialDrive m_drive;
  private final DoubleSolenoid shifter = new DoubleSolenoid(Constants.PCM.driveLow, Constants.PCM.driveHigh);

  private final WPI_TalonSRX m_leftSRX;
  private final WPI_TalonSRX m_rightSRX;

  private final WPI_VictorSPX m_leftSPX1;
  private final WPI_VictorSPX m_rightSPX1;

  private final WPI_VictorSPX m_leftSPX2;
  private final WPI_VictorSPX m_rightSPX2;

  private Gyro m_gyro;
  
  private boolean reverse = false;
  private boolean low = true;

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public Drivetrain(Gyro gyro) {
    m_gyro = gyro;
    
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
    

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(),
        m_leftSRX.getSelectedSensorPosition() / Constants.DriveConstants.ticksPerMeterLeft,
        m_rightSRX.getSelectedSensorPosition() / Constants.DriveConstants.ticksPerMeterRight);
        SmartDashboard.putNumber("Left Encoder", m_leftSRX.getSelectedSensorPosition());
        SmartDashboard.putNumber("Right Encoder", m_rightSRX.getSelectedSensorPosition());
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
        m_leftSRX.getSelectedSensorVelocity() / Constants.DriveConstants.ticksPerMeterLeft,
        m_rightSRX.getSelectedSensorVelocity() / Constants.DriveConstants.ticksPerMeterRight);
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
   * Reverses the drivetrain
   * @param reverse Reverse the drivetrain
   */
  public void reverse(final boolean reverse) {
    this.reverse = reverse;
  }

  /**
   * Checks if the drivetrain is reversed
   * @return True if the drivetrain is reversed
   */
  public boolean isReversed() {
    return reverse;
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    if (reverse) {
      fwd = -fwd;
    }
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Drives the robot using tank controls.
   *
   * @param left  the commanded left movement
   * @param right the commanded right movement
   */
  public void tankDrive(double left, double right) {
    tankDrive(left, right, true);
  }

  /**
   * Drives the robot using tank controls.
   *
   * @param left  the commanded left movement
   * @param right the commanded right movement
   * @param squareInputs square the input values
   */
  public void tankDrive(final double left, final double right, boolean squareInputs) {
    if (reverse) {
      m_drive.tankDrive(-right, -left, squareInputs);
    } else {
      m_drive.tankDrive(left, right, squareInputs);
    }
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
    double total = m_leftSRX.getSelectedSensorPosition(0) / Constants.DriveConstants.ticksPerMeterLeft;
    total += m_rightSRX.getSelectedSensorPosition(0) / Constants.DriveConstants.ticksPerMeterRight;
    return (total / 2);
  }

  /**
   * Sets the robot to low gear for a 2 speed gearbox
   */
  public void lowGear() {
		if (shifter.get() != DoubleSolenoid.Value.kForward) {
			shifter.set(DoubleSolenoid.Value.kForward);
    }
    low = true;
  }
  
  /**
   * Sets the robot to high gear for a 2 speed gearbox
   */
  public void highGear() {
		if (shifter.get() != DoubleSolenoid.Value.kReverse) {
			shifter.set(DoubleSolenoid.Value.kReverse);
    }
    low = false;
  }
}
