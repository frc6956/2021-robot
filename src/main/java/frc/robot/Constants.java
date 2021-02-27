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
  public final class CAN {
    public static final int leftDriveMotor = 1;
    public static final int leftDriveSPX1 = 11;
    public static final int leftDriveSPX2 = 3;
    public static final int rightDriveMotor = 2;
    public static final int rightDriveSPX1 = 12;
    public static final int rightDriveSPX2 = 4;
  }

  public final class PDP {
    public static final int leftDriveMotor = 0;
    public static final int leftDriveSPX1 = 1;
    public static final int leftDriveSPX2 = 2;
    public static final int rightDriveMotor = 15;
    public static final int rightDriveSPX1 = 14;
    public static final int rightDriveSPX2 = 13;
  }

  public static final class DriveConstants {

    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final double kTrackwidthMeters = 1.0744786401204909 / 2;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackwidthMeters);

    public static final double ksVolts = 1.23;
    public static final double kvVoltSecondsPerMeter = 4.63;
    public static final double kaVoltSecondsSquaredPerMeter = 0.407;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 1.79;

    // From old code
    public static final int kTicksPerRev = 4096;
    public static final double kWheelCircumference = .2 * Math.PI;
    public static final double ticksPerMeter = kTicksPerRev / kWheelCircumference;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 1;
  }

  // USB
  public final class USB {
      public static final int driveController = 0;
      public static final int operatorController = 1;
      public static final int driverJoyLeft = 2;
      public static final int driverJoyRight = 3;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}
