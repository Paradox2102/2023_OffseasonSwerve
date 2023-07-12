// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */

  WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(0);
  MaxSwerveModule m_frontRight = new MaxSwerveModule(Constants.k_FRDriveMotor, Constants.k_FRTurningMotor, 0);
  MaxSwerveModule m_frontLeft = new MaxSwerveModule(Constants.k_FLDriveMotor, Constants.k_FLTurningMotor, 0);
  MaxSwerveModule m_backRigh = new MaxSwerveModule(Constants.k_BRDriveMotor, Constants.k_BRTurningMotor, 0);
  MaxSwerveModule backLeft = new MaxSwerveModule(Constants.k_BLDriveMotor, Constants.k_BLTurningMotor, 0);
  
  public DriveSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
