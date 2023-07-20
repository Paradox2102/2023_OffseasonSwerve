// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
  private double m_azimuthZero = 0;
  private double m_angle = 0;
  private final static double k_p = 0.01;
  private final static double k_deadZone = 1.0;

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;
 
  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;
  
  // private final TalonSRX m_frontLeftEncoder = new TalonSRX(Constants.k_climber);
  // private final TalonSRX m_frontRightEncoder = new TalonSRX(Constants.k_climbFollower);
  // private final TalonSRX m_backLeftEncoder = new TalonSRX(Constants.k_intake);
  // private final TalonSRX m_backRightEncoder = new TalonSRX(Constants.k_scotty);

  private String m_smartDashboardName;
  private String m_name;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   *
   * @param driveMotorChannel   PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   */

  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      String name) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_driveMotor.restoreFactoryDefaults(); 
    m_turningMotor.restoreFactoryDefaults(); 
    m_driveMotor.setInverted(true);
    m_smartDashboardName = name + "Azimuth Zero";
    m_name = name; 

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = m_turningMotor.getEncoder();
    // m_frontLeftEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute); 
    // m_frontRightEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute); 
    // m_backLeftEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute); 
    // m_backRightEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute); 

    setPID(m_driveMotor.getPIDController(), Constants.k_driveF, Constants.k_driveP, Constants.k_driveI, Constants.k_driveIZone);
  }

  public void setPID(SparkMaxPIDController controller, double f, double p, double i, double iZone) {
    controller.setFF(f);
    controller.setP(p);
    controller.setI(i);
    controller.setIZone(iZone);
    controller.setD(0);
    controller.setOutputRange(-1, 1);
  }

  public void storeAzimuthZeroReference() {
    SmartDashboard.setPersistent(m_smartDashboardName);

    m_azimuthZero = getRawAngleDegrees();
    System.out.println(String.format("Zero = %f, Angle = %f", m_azimuthZero, getAngleDegrees())); 
    SmartDashboard.putNumber(m_smartDashboardName, m_azimuthZero);
    // System.out.println(SmartDashboard.getNumber(m_smartDashboardName, 0));
    SmartDashboard.setPersistent(m_smartDashboardName);
  }

  public void loadAndSetAzimuthZeroReference() {
    m_azimuthZero = SmartDashboard.getNumber(m_smartDashboardName, getAngleDegrees());
    SmartDashboard.putNumber(m_smartDashboardName, m_azimuthZero);
  }

  private double getAngleDegrees() {
    return (360.0 / 112 * m_turningEncoder.getPosition()) - m_azimuthZero;
    // return m_frontLeftEncoder.getSelectedPosition() - m_azimuthZero; 
  }

  private double getRawAngleDegrees() {
    return (360.0 / 112 * (m_turningEncoder.getPosition()));
    // return m_frontLeftEncoder.getSelectedPosition(); 
  }

  private double getVelocityMPS() {
    return m_driveEncoder.getVelocity() * Constants.k_driveTicksToMetersVelocity;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    Rotation2d angle = Rotation2d.fromDegrees(getAngleDegrees());
    return new SwerveModuleState(getVelocityMPS(), angle);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(getAngleDegrees()));
    setWheelAngle(state.angle.getDegrees());
    setSpeed(state.speedMetersPerSecond);
    SmartDashboard.putNumber(m_name + "Azimuth", state.angle.getDegrees());
  }

  private void setSpeed(double speed) {
    m_driveMotor.getPIDController().setReference(speed / Constants.k_turnTicksToDegreesVelocity, CANSparkMax.ControlType.kVelocity);
  }

  public double normalizeAngle(double angle) {
    double a = angle % 360;

    if (a > 180) {
      a -= 360;
    } else if (a < -180) {
      a += 360;
    }

    return a;
  }

  public void setWheelAngle(double angle) {
    m_angle = angle;
  }

  public void periodic() {
    double error = normalizeAngle(getState().angle.getDegrees() - m_angle);
    double power = -error * k_p;
    if (Math.abs(error) < k_deadZone) {
      power = 0;
    }
    m_turningMotor.set(power);
    // System.out.println(error + " " + m_angle + " " + k_deadZone);
    SmartDashboard.putNumber(m_name + "Angle", normalizeAngle(getState().angle.getDegrees()));
  }

  public void setBrakeMode(boolean brakeMode) {
    m_driveMotor.setIdleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);
  }
}