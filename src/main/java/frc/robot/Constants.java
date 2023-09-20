// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

  public static final double k_startAngleDegrees = 90;

  public static final double k_xFrontCameraOffsetInches = 0;
  public static final double k_yFrontCameraOffsetInches = 13.25;
  public static final double k_frontCameraAngle = 0;

  public static boolean k_isCubeMode = true;
  public static boolean k_hasGamePiece = false;
  public static ArmPosition k_armPosition = ArmPosition.NEUTRAL;

  // Non-Drive motors
  public static final int k_intakeMotor = 31;
  public static final int k_elevatorMotor = 30;
  public static final int k_elevatorFollower = 29;
  public static final int k_wristMotor = 28;
  public static final int k_topSwitch = 27;
  public static final int k_bottomSwitch = 26;

  // Elevator Constants
  public static final double k_minExtentInches = 0;
  public static final double k_maxExtentInches = 55;
  public static final double k_elevatorTicksToInches = 1;

  // Wrist Constants
  public static final double k_wristAngleTicksToDegrees = 1;
  public static final double k_minAngleDegrees = 0;
  public static final double k_maxAngleDegrees = 180;

  // Neutral Pose
  public static final double k_neutralElevatorInches = 0;
  public static final double k_neutralWristDegrees = 0;

  public enum ArmPosition {
    HIGH,
    MID,
    SINGLE,
    DOUBLE,
    GROUND,
    NEUTRAL
  }

  // Cube presets
  public static final class CubeConstants {
    // Intake
    public static final double k_intakePower = .5;
    public static final double k_outtakePower = -.5;
    public static final double k_intakeF = 0;

    // Elevator
    public static final double k_highElevatorInches = 0;
    public static final double k_midElevatorInches = 0;
    public static final double k_singleElevatorInches = 0;
    public static final double k_doubleElevatorInches = 0;
    public static final double k_groundElevatorInches = 0;

    // Wrist
    public static final double k_highWristDegrees = 0;
    public static final double k_midWristDegrees = 0;
    public static final double k_singleWristDegrees = 0;
    public static final double k_doubleWristDegrees = 0;
    public static final double k_groundWristDegrees = 0;
  }

  // Cone presets
  public static final class ConeConstants {
    // Intake
    public static final double k_intakePower = -CubeConstants.k_intakePower;
    public static final double k_outtakePower = -CubeConstants.k_outtakePower;
    public static final double k_intakeF = -CubeConstants.k_intakeF;

    // Elevator
    public static final double k_highElevatorInches = 0;
    public static final double k_midElevatorInches = 0;
    public static final double k_singleElevatorInches = 0;
    public static final double k_doubleElevatorInches = 0;
    public static final double k_groundElevatorInches = 0;

    // Wrist
    public static final double k_highWristDegrees = 0;
    public static final double k_midWristDegrees = 0;
    public static final double k_singleWristDegrees = 0;
    public static final double k_doubleWristDegrees = 0;
    public static final double k_groundWristDegrees = 0;
  }

  // DRIVETRAIN SPARK MAX IDs
  // Front of robot is opposite battery
  public static final int k_FRDriveMotor = 1; // Front Right // 3
  public static final int k_FLDriveMotor = 3; // Front Left // 1
  public static final int k_BRDriveMotor = 5; // Back Right // 7
  public static final int k_BLDriveMotor = 7; // Back Left // 5

  public static final int k_FRTurningMotor = 2; // 4
  public static final int k_FLTurningMotor = 4; // 2
  public static final int k_BRTurningMotor = 6; // 8
  public static final int k_BLTurningMotor = 8; // 6

  public static final boolean k_gyroReversed = false;

  // Angular offsets of the modules relative to the chassis in radians

  // Bolt
  // public static final double k_FLOffset = 1.09 - (Math.PI / 2); // 5.425 - (Math.PI / 2);
  // public static final double k_FROffset = 6.06; // 1.171;
  // public static final double k_BLOffset = 6.22 +(Math.PI); // 6.173 + (Math.PI);
  // public static final double k_BROffset = 4.75 + (Math.PI / 2); // 3.243 + (Math.PI / 2);

  // Byte
  public static final double k_FLOffset = 5.425 - (Math.PI / 2);
  public static final double k_FROffset = 1.171;
  public static final double k_BLOffset = 6.173 + (Math.PI);
  public static final double k_BROffset = 3.243 + (Math.PI / 2);

  public static final boolean k_isGyroReversed = true;

  public static final int k_drivingMotorPinionTeeth = 14;

  public static final double k_driveWidth = Units.inchesToMeters(26.5);
  public static final double k_driveLength = Units.inchesToMeters(26.5);
  public static final double k_wheelDiameterMeters = .0762;
  public static final double k_drivingMotorReduction = (45.0 * 22) / (k_drivingMotorPinionTeeth * 15);

  public static final double k_driveTicksToMetersVelocity = ((k_wheelDiameterMeters * Math.PI)
      / k_drivingMotorReduction) / 60.0;
  public static final double k_driveTicksToMetersPosition = (k_wheelDiameterMeters * Math.PI) / k_drivingMotorReduction;
  public static final double k_turnTicksToDegreesVelocity = (2 * Math.PI) / 60.0;
  public static final double k_turnTicksToMetersPosition = (2 * Math.PI);

  public static final double k_turningEncoderPositionPIDMinInput = 0; // radians
  public static final double k_turningEncoderPositionPIDMaxInput = k_turnTicksToMetersPosition; // radians

  public static final boolean k_turningEncoderInverted = true;

  public static final double k_freeSpeedRPM = 5676;
  public static final double k_drivingMotorFreeSpeedRps = k_freeSpeedRPM / 60.0;
  public static final double k_wheelCircumferenceMeters = k_wheelDiameterMeters * Math.PI;
  public static final double k_driveWheelFreeSpeedRps = (k_drivingMotorFreeSpeedRps * k_wheelCircumferenceMeters)
      / k_drivingMotorReduction;

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
  public static final double k_maxSpeedMetersPerSecond = 3;
  public static final double k_maxDriveAcceleration = 3;
  public static final double k_maxAngularSpeed = Math.PI; // radians per second
  public static final double k_maxAngularAcceleration = Math.PI;

  public static final double k_directionSlewRate = 1.2; // radians per second
  public static final double k_magnitudeSlewRate = 1.8; // percent per second (1 = 100%)
  public static final double k_rotationalSlewRate = 2.0; // percent per second (1 = 100%)

  public static final SwerveModuleState[] k_defaultState = {
      new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
      new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4)),
      new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4)),
      new SwerveModuleState(0, new Rotation2d(Math.PI / 4))
  };

  public static final double k_driveDeadband = 0.05;
  public static final TrapezoidProfile.Constraints k_thetaControllerConstraints = new TrapezoidProfile.Constraints(
      k_maxSpeedMetersPerSecond, k_maxAngularAcceleration);

}