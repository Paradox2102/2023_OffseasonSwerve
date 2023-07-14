// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  // public static class OperatorConstants {
  // public static final int kDriverControllerPort = 0;
  // }

  // DRIVETRAIN SPARK MAX IDs
  // Front of robot is opposite battery
  public static final int k_FLDriveMotor = 1; // Front Left
  public static final int k_FRDriveMotor = 3; // Front Right
  public static final int k_BLDriveMotor = 5; // Back Left
  public static final int k_BRDriveMotor = 7; // Back Right

  public static final int k_FLTurningMotor = 2;
  public static final int k_FRTurningMotor = 4;
  public static final int k_BLTurningMotor = 6;
  public static final int k_BRTurningMotor = 8;

  public static final boolean k_isGyroReversed = false;

  public static final double k_driveWidth = Units.inchesToMeters(26.5);
  public static final double k_driveLength = Units.inchesToMeters(26.5);

  public static final double k_driveTicksToMetersVelocity = 1; // to be tuned
  public static final double k_driveTicksToMetersPosition = 1; // to be tuned
  public static final double k_turnTicksToMetersVelocity = 1; // to be tuned
  public static final double k_turnTicksToMetersPosition = 362.2;

  // Swerve Module Drive PID
  public static final double k_driveP = 1;
  public static final double k_driveI = 1;
  public static final double k_driveD = 1;

  // Swerve Module Turn PID
  public static final double k_turnP = 1;
  public static final double k_turnI = 1;
  public static final double k_turnD = 1;

  // Driving Constants 
  public static final double k_maxSpeedMetersPerSecond = 4.8;
  public static final double k_maxAngularSpeed = 2 * Math.PI; // radians per second
  
  public static final double k_directionSlewRate = 1.2; // radians per second
  public static final double k_magnitudeSlewRate = 1.8; // percent per second (1 = 100%)
  public static final double k_rotationalSlewRate = 2.0; // percent per second (1 = 100%)

  public static final double k_driveDeadband = 0.05;

}
