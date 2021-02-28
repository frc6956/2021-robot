/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.*;
import frc.robot.Constants;

/**
 * Creates a new Color Spinner .
 */

public class ColorSpinner extends SubsystemBase {
   private String[] clrOrder = {"red", "green", "blue", "yellow"};
   String matchedClr = "not Spinning";
   private WPI_TalonSRX spinnerMotor = new WPI_TalonSRX(Constants.CAN.spinner);
   private ColorSensorV3 clrSensor = new ColorSensorV3(I2C.Port.kOnboard);
   private ColorMatch clrMatch = new ColorMatch();
   private boolean rotating = false;
   private boolean clockwise = false;
   private boolean up = false;
   private Color matchedColor;
   private int index = -1; 
   private double rotations = 0;

   DoubleSolenoid doubleSolenoid = new DoubleSolenoid(Constants.PCM.spinnerUp, Constants.PCM.spinnerDown);
   
  /**
   * adds color for the color sensor to compare agianst
   */
  public ColorSpinner() {
    clrMatch.addColorMatch(Color.kAqua);
    clrMatch.addColorMatch(Color.kRed);
    clrMatch.addColorMatch(Color.kYellow);
    clrMatch.addColorMatch(Color.kLime);
    spinnerMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Spinner Speed", getRPM());
   /* rotating();
    SmartDashboard.putNumber("Rotations", rotations);
    displayColor();*/
    desiredColor();
  }

  public void resetRotations() {
    rotations = 0;
  }


/**
 * Gets the color that the control panel needs to be set to for stage 3
 */
  public void desiredColor() {
    SmartDashboard.putString("Required Color", DriverStation.getInstance().getGameSpecificMessage());
  }

  /**
   * Tests to see if the color seem by the color sensor is the next predicted color
   * If it is increase amount of rotation made if not
   * Do nothing just recheck color in next iteration
   */
  public void rotating() {
    Color seenColor = clrMatch.matchClosestColor(clrSensor.getColor()).color;
    int seenIndex = getIndex(seenColor);
    if(rotating && index!=seenIndex && (clockwise && (index+1)%4 == seenIndex || !clockwise && index == (seenIndex+1)%4)) {
      setColor();
      matchedClr = clrOrder[index];
      rotations +=.125;
    }
  }

  public void setColor() {
    matchedColor = clrMatch.matchClosestColor(clrSensor.getColor()).color;
    index = getIndex(matchedColor);
  }

  public int getIndex(Color clr) {
    if(clr.equals(Color.kLime)){
      return 1;
    }
    else if(clr.equals(Color.kAqua)){
      return 2;
    }
    else if(clr.equals(Color.kYellow)){
      return 3;
    }
    else if(clr.equals(Color.kRed)){
      return 0;
    }
    return -1;
  }


  public void displayRGB() {
    SmartDashboard.putNumber("Red:", clrSensor.getRed());
    SmartDashboard.putNumber("Green:", clrSensor.getGreen());
    SmartDashboard.putNumber("Blue:", clrSensor.getBlue());
  }

  public void displayColor() {
    SmartDashboard.putString("Color", matchedClr);
    SmartDashboard.putNumber("Color Confidnece", clrMatch.matchClosestColor(clrSensor.getColor()).confidence);
  }


  /**
   * Will bring up color Spinner
   */
  public void up() {
    if (doubleSolenoid.get() != DoubleSolenoid.Value.kReverse) {
      doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
      resetRotations();
      up = true;
    }
  }

  /**
   * Will put down color Spinner
   */
  public void down() {
    if (doubleSolenoid.get() != DoubleSolenoid.Value.kForward) {
      doubleSolenoid.set(DoubleSolenoid.Value.kForward);
      up= false;
    }
  }

  public void setWheelSpeed(double speed) {
   /* if(Math.abs(speed)>.001) {
          rotating = true;
          setColor();
          if(speed>0) {
            clockwise = true;
          }
          else {
            clockwise = false;
          }
    }
    else {
      rotating = false;
      matchedClr = "not Spinning";
    }*/
    if(up) {
      spinnerMotor.set(speed);
    }
  }

  public void setRPM(double rpm) {
    double voltage = rpm / Constants.ColorSpinner.kRPMPerVolt;
    if(up) {
      spinnerMotor.setVoltage(voltage);
    }
  }

  public double getRPM() {
    return spinnerMotor.getSelectedSensorVelocity() * 10 * 60 / Constants.ColorSpinner.kTicksPerRev;
  }
}
