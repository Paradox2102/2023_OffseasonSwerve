// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Drivetrain;

public class DriveSubsystem extends SubsystemBase {
  private final Drivetrain m_swerve;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_swerve = new Drivetrain();
  }

  public Drivetrain getSwerve() {
    return m_swerve;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // m_FLmotor.periodic();
    // m_FRmotor.periodic();
    // m_BLmotor.periodic();
    // m_BRmotor.periodic();

    m_swerve.updateOdometry();
    Pose2d pos = m_swerve.m_odometry.getPoseMeters();
    SmartDashboard.putNumber("x position", pos.getX());
    SmartDashboard.putNumber("y position", pos.getY());
    SmartDashboard.putNumber("gyro", m_swerve.m_gyro.getAngle());

    SmartDashboard.putNumber("ATurn FR", m_swerve.m_frontRight.getAngle());
    SmartDashboard.putNumber("ATurn FL", m_swerve.m_frontLeft.getAngle());
    SmartDashboard.putNumber("ATurn BR", m_swerve.m_backRight.getAngle());
    SmartDashboard.putNumber("ATurn BL", m_swerve.m_backLeft.getAngle());

    SmartDashboard.putNumber("RTurn FR", m_swerve.m_frontRight.RgetAngle());
    SmartDashboard.putNumber("RTurn FL", m_swerve.m_frontLeft.RgetAngle());
    SmartDashboard.putNumber("RTurn BR", m_swerve.m_backRight.RgetAngle());
    SmartDashboard.putNumber("RTurn BL", m_swerve.m_backLeft.RgetAngle());
  }

  // public void storeAzimuthZeroReference() {
  //   m_FLmotor.storeAzimuthZeroReference();
  //   m_FRmotor.storeAzimuthZeroReference();
  //   m_BLmotor.storeAzimuthZeroReference();
  //   m_BRmotor.storeAzimuthZeroReference();
  // }

  // public void loadAndSetAzimuthZeroReference() {
  //   m_FLmotor.loadAndSetAzimuthZeroReference();
  //   m_FRmotor.loadAndSetAzimuthZeroReference();
  //   m_BLmotor.loadAndSetAzimuthZeroReference();
  //   m_BRmotor.loadAndSetAzimuthZeroReference();
  // }

  // public void setBrakeMode(boolean isBrake) {
  //   m_FLmotor.setBrakeMode(isBrake);
  //   m_FRmotor.setBrakeMode(isBrake);
  //   m_BLmotor.setBrakeMode(isBrake);
  //   m_BRmotor.setBrakeMode(isBrake);
  // }

  public Pose2d getPose() {
    return m_swerve.m_odometry.getPoseMeters();
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, 3);
    m_swerve.m_frontLeft.setDesiredState(desiredStates[0]);
    m_swerve.m_frontRight.setDesiredState(desiredStates[1]);
    m_swerve.m_backLeft.setDesiredState(desiredStates[2]);
    m_swerve.m_backRight.setDesiredState(desiredStates[3]);
  }

  public void resetOdometry(Pose2d pose) {
    m_swerve.m_odometry.resetPosition(Rotation2d.fromDegrees(m_swerve.m_gyro.getAngle()), m_swerve.m_modulePositions, new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
  }
}