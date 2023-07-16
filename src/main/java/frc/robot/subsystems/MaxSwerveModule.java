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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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

  private double m_chassisRotationOffset;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  
  public MaxSwerveModule(int driveID, int turnID, double chassisRotationOffset) {
    // Initialize the motors
    m_driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    m_turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);

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
    m_turnEncoder.setVelocityConversionFactor(Constants.k_turnTicksToDegreesVelocity);
    // m_driveEncoder.setInverted(false);
    // m_turnEncoder.setInverted(true);

    // Allow turn motor to go from 359 degrees to 0 degrees directly and not rotate 359 degrees the other direction
    m_turnPID.setPositionPIDWrappingEnabled(true);
    m_turnPID.setPositionPIDWrappingMaxInput(2 * Math.PI);
    m_turnPID.setPositionPIDWrappingMinInput(0);

    // Set the PIDs
    m_drivePID.setP(Constants.k_driveP);
    // m_drivePID.setI(Constants.k_driveI);
    // m_drivePID.setD(Constants.k_driveD);

    m_turnPID.setP(Constants.k_turnP);
    // m_turnPID.setI(Constants.k_turnI);
    // m_turnPID.setD(Constants.k_turnD);

    // Save the config onto the motors
    m_driveMotor.burnFlash();
    m_turnMotor.burnFlash();

    m_chassisRotationOffset = chassisRotationOffset;
    m_desiredState.angle = new Rotation2d(m_turnEncoder.getPosition());
    m_driveEncoder.setPosition(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(m_turnEncoder.getPosition() - m_chassisRotationOffset));
  }

  // Set the swerve module to their desired states (Velocity at which the wheel is moving and the angle it is turned)
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisRotationOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, new Rotation2d(m_turnEncoder.getPosition()));

    // Change the state of the module
    m_drivePID.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turnPID.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
  }

  public void drive1(double s) {
    m_driveMotor.set(s);
  }

  public void turn1(double s) {
    m_turnMotor.set(s);
  }

  public double getAngle() {
    return m_turnEncoder.getPosition();
  }

  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(),
        new Rotation2d(m_turnEncoder.getPosition() - m_chassisRotationOffset));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println("MAXSwerveModule Reading"); 
  }
}
