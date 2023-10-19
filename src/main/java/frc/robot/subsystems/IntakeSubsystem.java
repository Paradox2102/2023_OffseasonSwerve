// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private double m_power = 0;
  private TalonFX m_motor = new TalonFX(Constants.k_intakeMotor, "Default Name");
  private final double k_stallSpeed = .075;
  private Timer m_stallTimer = new Timer();

  public enum IntakeType {INTAKE, OUTTAKE, STOP}

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_stallTimer.reset();
    m_stallTimer.start();
    setBrakeMode(true);
  }

  public void setPower(double power) {
    Logger.log("IntakeSubsystem", 1, String.format("setPower: %f", power));
    m_power = power;
  }

  public void intake() {
    m_stallTimer.reset();
    m_power = Constants.k_isCubeMode ? Constants.CubeConstants.k_intakePower : Constants.ConeConstants.k_intakePower;
  }

  public void outtake() {
    m_power = Constants.k_isCubeMode ? Constants.CubeConstants.k_outtakePower : Constants.ConeConstants.k_outtakePower;
  }

  public void stop() {
    m_power = Constants.k_isCubeMode ? 0 : -k_stallSpeed;
  }

  public void setBrakeMode(boolean brake) {
    m_motor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
  }

  // Lower power if game piece is acquired
  private void isIntakeStalled() {
    double speed = m_motor.getSelectedSensorVelocity();
    if (m_stallTimer.get() > 1) {
      if (Math.abs(speed) < 1) {
        stop();
        m_stallTimer.reset();
      } else {
        m_stallTimer.reset();
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    isIntakeStalled();
    double power = k_stallSpeed + m_power;
    m_motor.set(ControlMode.PercentOutput, power);
    SmartDashboard.putNumber("Intake Speed", m_motor.getSelectedSensorVelocity());
  }
}
