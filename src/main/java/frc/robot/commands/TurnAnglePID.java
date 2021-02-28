/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html

/**
 * Angular rotation of the robot, uses a PID loop to correct for over rotation
 */
public class TurnAnglePID extends PIDCommand {
  /**
   * Creates a new TurnAnglePID.
   */
  private final double m_turnAngle;
  private final Gyro m_gyro;
  public TurnAnglePID(final Drivetrain drivetrain, final Gyro gyro, final double turnAngle) {
    super(
        // The controller that the command will use
        new PIDController(SmartDashboard.getNumber("KP", .5), SmartDashboard.getNumber("KI", 0), SmartDashboard.getNumber("KD", 0) ),
        // This should return the measurement
        () -> gyro.getAngle(),
        // This should return the setpoint (can also be a constant)
        () -> turnAngle,
        // This uses the output
        output -> {
          // Use the output here
          SmartDashboard.putNumber("PID Angle", output);
          if(output > .5) {
            drivetrain.tankDrive(.5, -.5, false);
          } else if(output < -.5) {
            drivetrain.tankDrive(-.5, .5, false);
          } else {
            drivetrain.tankDrive(output, -output, false);
          }
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(drivetrain);

    m_turnAngle = turnAngle;
    m_gyro = gyro;

    SmartDashboard.putNumber("KP", SmartDashboard.getNumber("KP", 0));
    SmartDashboard.putNumber("KI", SmartDashboard.getNumber("KI", 0));
    SmartDashboard.putNumber("KD", SmartDashboard.getNumber("KD", 0));
    SmartDashboard.putNumber("I Limit", SmartDashboard.getNumber("I Limit", 0));
  }

  @Override
  public void initialize() {
    super.initialize();
    m_gyro.reset();
    m_controller.setP(SmartDashboard.getNumber("KP", 0));
    m_controller.setP(SmartDashboard.getNumber("KI", 0));
    m_controller.setP(SmartDashboard.getNumber("KD", 0));
    m_controller.setIntegratorRange(-SmartDashboard.getNumber("I Limit", 0), SmartDashboard.getNumber("I Limit", 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_gyro.getAngle() - m_turnAngle) < 1;
  }
}
