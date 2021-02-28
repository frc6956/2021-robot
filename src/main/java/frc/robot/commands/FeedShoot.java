/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

  /**
   * Creates a new FeedShoot.
   */
public class FeedShoot extends CommandBase {
  private final Shooter mShooter;
  private final Feeder mFeeder;
  private final Conveyor mConveyor;
  private final double shootSpeed;
  private final double feedSpeed;
  private final double conSpeed;
  private final double timeout;
  private final Timer timer;

  public FeedShoot(final Shooter shooter, final Feeder feeder, final Conveyor conveyor, final double shooterSpeed, final double feederSpeed, final double conveyorSpeed, final double timeout) {
    mShooter = shooter;
    mFeeder = feeder;
    mConveyor = conveyor;
    shootSpeed = shooterSpeed;
    feedSpeed = feederSpeed;
    conSpeed = conveyorSpeed;
    this.timeout = timeout;
    timer = new Timer();

    addRequirements(mShooter);
    addRequirements(mFeeder);
    addRequirements(mConveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mShooter.setShooterSpeed(shootSpeed);
    mFeeder.setFeedSpeed(-feedSpeed);
    mConveyor.setConveyorSpeed(-conSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    mShooter.setShooterRPM(0);
    mFeeder.setFeedSpeed(0);
    mConveyor.setConveyorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasPeriodPassed(timeout);
  }
}
