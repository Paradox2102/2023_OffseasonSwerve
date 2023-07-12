// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MaxSwerveModule extends SubsystemBase {
  /** Creates a new MaxSwerveModule. */

  // Create motors on the swerve module
  private CANSparkMax m_driveMotor;
  private CANSparkMax m_turnMotor;

  // PID Objects
  private final SparkMaxPIDController m_drivePID;
  private final SparkMaxPIDController m_turnPID;

  // Create encoder objects
  private final RelativeEncoder m_driveEncoder;
  private final AbsoluteEncoder m_turnEncoder;


  
  public MaxSwerveModule(int driveID, int turnID, double chassisAngularOffset) {
    // Initialize the motors
    m_driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    m_turnMotor = new CANSparkMax(turnID, MotorType.kBrushed);

    // Reset Motors
    m_driveMotor.restoreFactoryDefaults();
    m_turnMotor.restoreFactoryDefaults();

    // Initializing other variables
    m_driveEncoder = m_driveMotor.getEncoder();
    m_turnEncoder = m_turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

    m_drivePID = m_driveMotor.getPIDController();
    m_turnPID = m_turnMotor.getPIDController();
    m_drivePID.setFeedbackDevice(m_driveEncoder);
    m_turnPID.setFeedbackDevice(m_turnEncoder);
    m_driveEncoder.setPositionConversionFactor(Constants.k_driveTicksToMetersPosition);
    m_driveEncoder.setVelocityConversionFactor(Constants.k_driveTicksToMetersVelocity);
    m_turnEncoder.setPositionConversionFactor(Constants.k_turnTicksToMetersPosition);
    m_turnEncoder.setVelocityConversionFactor(Constants.k_turnTicksToMetersVelocity);
    m_driveEncoder.setInverted(false);
    m_turnEncoder.setInverted(true);

    // Allow turn motor to go from 359 degrees to 0 degrees directly and not rotate 359 degrees the other direction
    m_turnPID.setPositionPIDWrappingEnabled(true);
    m_turnPID.setPositionPIDWrappingMaxInput(2 * Math.PI);
    m_turnPID.setPositionPIDWrappingMinInput(0);

    // Save the config onto the motors
    m_driveMotor.burnFlash();
    m_turnMotor.burnFlash();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
