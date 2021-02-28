/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
/**
 * Creates a new DriveDistance.
 */

public class DriveDistance extends CommandBase {
  private  Drivetrain mdrivetrain;
  private  double requestedDistance;

  public DriveDistance(final Drivetrain drivetrain, final double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    mdrivetrain = drivetrain;
    requestedDistance = distance;
    addRequirements(mdrivetrain);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mdrivetrain.resetDistanceTravelled();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(requestedDistance >= 0) {
      mdrivetrain.tankDrive(0.4, 0.4, false);
    } else {
      mdrivetrain.tankDrive(-0.4, -0.4, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    mdrivetrain.tankDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(requestedDistance >= 0) {
      if(mdrivetrain.getDistanceTravelled() >= requestedDistance) {
        return true;
      } else {
        return false;
      }
    } else {
      if(mdrivetrain.getDistanceTravelled() <= requestedDistance) {
        return true;
      } else {
        return false;
      }
    }
    //return false;
  }
}