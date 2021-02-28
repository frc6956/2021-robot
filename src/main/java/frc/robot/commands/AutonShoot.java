/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
/**
 * Creates a new AutonShoot.
 */

public class AutonShoot extends CommandBase {
  private final Intake mIntake;
  private final Conveyor mConveyor;
  private final Feeder mFeeder;
  private final Shooter mShooter;
  private double inSpeed;
  private double conSpeed;
  private double feedSpeed;
  private double shootSpeed;

  public AutonShoot(final Intake intake, final Conveyor conveyor, final Feeder feeder, final Shooter shooter,
    final double intakeSpeed, final double conveyorSpeed, final double feederSpeed, final double shooterSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    mIntake = intake;
    mConveyor = conveyor;
    mFeeder = feeder;
    mShooter = shooter;
    inSpeed = intakeSpeed;
    conSpeed = conveyorSpeed;
    feedSpeed = feederSpeed;
    shootSpeed = shooterSpeed;
    addRequirements(mIntake);
    addRequirements(mConveyor);
    addRequirements(mFeeder);
    addRequirements(mShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mShooter.setShooterSpeed(shootSpeed);
    mFeeder.setFeedSpeed(feedSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
