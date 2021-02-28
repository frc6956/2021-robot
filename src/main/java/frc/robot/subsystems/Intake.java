/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The mouth of the Hungry Hippo
 */
public class Intake extends SubsystemBase {
  
  private WPI_VictorSPX m_motor;

  public Intake() {
    m_motor = new WPI_VictorSPX(Constants.CAN.intakeMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the intake speed
   * @param speed Percentage of bus voltage
   */
  public void setIntakeSpeed(double speed) {
    m_motor.set(speed);
  }

}
