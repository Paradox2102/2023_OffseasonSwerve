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
  public static final int k_FRDriveMotor = 1; // Front Right   // 3
  public static final int k_FLDriveMotor = 3; // Front Left   // 1
  public static final int k_BRDriveMotor = 5; // Back Right  // 7
  public static final int k_BLDriveMotor = 7; // Back Left  // 5

  public static final int k_FRTurningMotor = 2;     // 4
  public static final int k_FLTurningMotor = 4;    // 2
  public static final int k_BRTurningMotor = 6;   // 8
  public static final int k_BLTurningMotor = 8;  // 6

  public static final boolean k_gyroReversed = false;

    // Angular offsets of the modules relative to the chassis in radians
    public static final double k_FLOffset = 5.425 - (Math.PI / 2);
    public static final double k_FROffset = 1.183;
    public static final double k_BLOffset = 6.161 + (Math.PI);
    public static final double k_BROffset = 3.248 + (Math.PI / 2);

  public static final boolean k_isGyroReversed = false;

  public static final int k_drivingMotorPinionTeeth = 14;

  public static final double k_driveWidth = Units.inchesToMeters(26.5);
  public static final double k_driveLength = Units.inchesToMeters(26.5);
  public static final double k_wheelDiameterMeters = .0762;
  public static final double k_drivingMotorReduction = (45.0 * 22) / (k_drivingMotorPinionTeeth * 15);

  public static final double k_driveTicksToMetersVelocity = ((k_wheelDiameterMeters * Math.PI) / k_drivingMotorReduction) / 60.0;
  public static final double k_driveTicksToMetersPosition = (k_wheelDiameterMeters * Math.PI) / k_drivingMotorReduction;
  public static final double k_turnTicksToDegreesVelocity = (2 * Math.PI) / 60.0;
  public static final double k_turnTicksToMetersPosition = (2 * Math.PI); 

  public static final double k_turningEncoderPositionPIDMinInput = 0; // radians
  public static final double k_turningEncoderPositionPIDMaxInput = k_turnTicksToMetersPosition; // radians
  
  public static final boolean k_turningEncoderInverted = true;


  public static final double k_freeSpeedRPM = 5676;
  public static final double k_drivingMotorFreeSpeedRps = k_freeSpeedRPM / 60.0;
  public static final double k_wheelCircumferenceMeters = k_wheelDiameterMeters * Math.PI;
  public static final double k_driveWheelFreeSpeedRps = (k_drivingMotorFreeSpeedRps * k_wheelCircumferenceMeters) / k_drivingMotorReduction;

  // Swerve Module Drive PID
  public static final double k_driveP = 0.04;
  public static final double k_driveI = 0;
  public static final double k_driveD = 0;
  public static final double k_driveFF = 1 / k_driveWheelFreeSpeedRps;
  public static final double k_drivingMinOutput = -1;
  public static final double k_drivingMaxOutput = 1;

  // Swerve Module Turn PID
  public static final double k_turnP = 1;
  public static final double k_turnI = 0;
  public static final double k_turnD = 0;
  public static final double k_turnFF = 0;
  public static final double k_turningMinOutput = -1;
  public static final double k_turningMaxOutput = 1;

  public static final int k_driveMotorCurrentLimit = 50; // amps
  public static final int k_turnMotorCurrentLimit = 20; // amps

  // Driving Constants 
  public static final double k_maxSpeedMetersPerSecond = 4.8;
  public static final double k_maxAngularSpeed = 2 * Math.PI; // radians per second
  
  public static final double k_directionSlewRate = 1.2; // radians per second
  public static final double k_magnitudeSlewRate = 1.8; // percent per second (1 = 100%)
  public static final double k_rotationalSlewRate = 2.0; // percent per second (1 = 100%)

  public static final double k_driveDeadband = 0.05;

} 