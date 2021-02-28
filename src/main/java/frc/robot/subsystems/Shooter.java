/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The 2 big angled wheels.
 */
public class Shooter extends SubsystemBase {
  
  private final WPI_TalonSRX m_left;
  private final WPI_TalonSRX m_right;

  public Shooter() {
      m_left = new WPI_TalonSRX(Constants.CAN.shooterLeft);
      m_right = new WPI_TalonSRX(Constants.CAN.shooterRight);

      m_left.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
      m_right.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

      m_right.setInverted(true);

      double p = 0.8;
      m_left.config_kP(0, p);
      m_right.config_kP(0, p);

      m_left.configClosedloopRamp(1.0);
      m_right.configClosedloopRamp(1.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Shooter Speed", getLeftShooterRPM());
    SmartDashboard.putNumber("Right Shooter Speed", getRightShooterRPM());
  }

  public void setShooterSpeed(double speed) {
    m_left.set(speed);
    m_right.set(speed);
  }

  public void setShooterRPM(double rpm) {
    double output = rpm * Constants.Shooter.ticsPerRev / 10 / 60 * 1.13; // why is the 1.13 correction needed???
    m_left.set(ControlMode.Velocity, output);
    m_right.set(ControlMode.Velocity, output);
  }

  public double getLeftShooterRPM() {
    return m_left.getSelectedSensorVelocity() * 10 * 60 / Constants.Shooter.ticsPerRev;
  }

  public double getRightShooterRPM() {
    return m_right.getSelectedSensorVelocity() * 10 * 60 / Constants.Shooter.ticsPerRev;
  }
}
