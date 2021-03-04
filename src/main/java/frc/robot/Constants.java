// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  // USB
  public final class USB {
      public static final int driveController = 0;
      public static final int operatorController = 1;
      public static final int driverJoyLeft = 2;
      public static final int driverJoyRight = 3;
  }

  // CAN
  public final class CAN {
      public static final int leftDriveMotor = 1;
      public static final int leftDriveSPX1 = 11;
      public static final int leftDriveSPX2 = 3;
      public static final int rightDriveMotor = 2;
      public static final int rightDriveSPX1 = 12;
      public static final int rightDriveSPX2 = 4;
      public static final int intakeMotor = 5;
      public static final int conveyorMotor = 7;
      public static final int feederMotor = 6;
      public static final int shooterLeft = 5;
      public static final int shooterRight = 4;
      public static final int spinner = 3;
  }

  public final class PDP {
      public static final int leftDriveMotor = 0;
      public static final int leftDriveSPX1 = 1;
      public static final int leftDriveSPX2 = 2;
      public static final int rightDriveMotor = 15;
      public static final int rightDriveSPX1 = 14;
      public static final int rightDriveSPX2 = 13;
      public static final int intakeMotor = 5;
      public static final int conveyorMotor = 6;
      public static final int feederMotor = 6;
      public static final int shooterLeft = 5;
      public static final int shooterRight = 4;
      public static final int spinner = 3;
  }

  //PCM
  public final class PCM {
      public static final int spinnerUp = 1;
      public static final int spinnerDown = 0;
      public static final int slideIn = 6;
      public static final int slideOut = 7;
      public static final int driveHigh = 3;
      public static final int driveLow = 2;
  }

  public final class Shooter {
      // 360 ticks per revolution * 4 quadrature ticks
      public static final int ticsPerRev = 360 * 4;
      public static final int kMaxRPM = 4500;
  }

  public final class ColorSpinner {
      // 7 ticks per revolution * 4 quadrature ticks * 4:1 gear reduction
      public static final int kTicksPerRev = 7 * 4 * 4;
      public static final int kRPMPerVolt = 475 / 4;
  }

  public static final class DriveConstants {

    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    //public static final double kTrackwidthMeters = 1.0744786401204909;
    public static final double kTrackwidthMeters = 0.85;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackwidthMeters);

    public static final double ksVolts = 1.23;
    public static final double kvVoltSecondsPerMeter = 4.63;
    public static final double kaVoltSecondsSquaredPerMeter = 0.407;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 1.79;

    // Encoder conversion
    public static final int kTicksPerRev = 4096;
    public static final double kLeftWheelDiameter = 0.181;
    public static final double kRightWheelDiameter = 0.178;
    public static final double ticksPerMeterLeft = kTicksPerRev / (kLeftWheelDiameter * Math.PI);
    public static final double ticksPerMeterRight = kTicksPerRev / (kRightWheelDiameter * Math.PI);
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}
