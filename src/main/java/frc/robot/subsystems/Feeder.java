/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
  
/**
 * 4in wheels on top of robot
 */

public class Feeder extends SubsystemBase {

private WPI_VictorSPX m_motor;

  public Feeder() {
    m_motor = new WPI_VictorSPX(Constants.CAN.feederMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the speed of the robot
   * @param speed Percentage of bus voltage
   */
  public void setFeedSpeed(double speed) {
    m_motor.set(speed);
  }
}
