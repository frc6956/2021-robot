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
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html

  /**
   * Drives the robot forward based in inches, and corrects using as PID loop
   */

public class DriveDistancePID extends PIDCommand {
  private final Drivetrain m_drivetrain;
  private final Gyro m_gyro;
  private final double m_requestedDistance;
  public DriveDistancePID(final Drivetrain drivetrain, final Gyro pigeon, final double distance) {
    super(
        // The controller that the command will use
        new PIDController(0.05, 0, 0),
        // This should return the measurement
        () -> pigeon.getAngle(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          drivetrain.tankDrive(0.5 - output, 0.5 + output, false);
        });

        m_drivetrain = drivetrain;
        m_gyro = pigeon;
        m_requestedDistance = distance;
    
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
    super.initialize();
    m_drivetrain.resetDistanceTravelled();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_drivetrain.getDistanceTravelled() > m_requestedDistance);
  }
}
