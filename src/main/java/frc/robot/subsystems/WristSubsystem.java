// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
  private TalonFX m_motor = new TalonFX(Constants.k_wristMotor, "Default Name");
  private final double k_P = .01;
  private final double k_F = .1;
  private final double k_deadzone = 2;
  private final double k_minPower = .1;
  private double m_targetAngleDegrees = 0;
  private double m_power = 0;

  /** Creates a new WristSubsystem. */
  public WristSubsystem() {
    setBrakeMode(true);
  }

  public void setAngleDegrees(double degree) {
    m_targetAngleDegrees = degree;
  }

  public void manualControl(double power) {
    m_power = power;
    m_targetAngleDegrees = getAngleDegrees();
  }

  public void setPower(double power) {
    m_motor.set(ControlMode.PercentOutput, power);
  }

  public void setBrakeMode(boolean brake) {
    m_motor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
  }

  private double getAngleDegrees() {
    return m_motor.getSelectedSensorPosition() * Constants.k_wristAngleTicksToDegrees;
  }

  private void checkLimits() {
    double angle = getAngleDegrees();
    if (m_power > 0 && angle >= Constants.k_maxAngleDegrees) {
      m_power = 0;
    } else if (m_power < 0 && angle <= Constants.k_minAngleDegrees) {
      m_power = 0;
    }
  }

  private void runP() {
    double distance = m_targetAngleDegrees - getAngleDegrees();
    m_power = k_P * distance;
    if (distance < k_deadzone) {
      m_power = k_F;
    } else if (m_power < k_minPower) {
      m_power = k_minPower * Math.signum(distance);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // runP();
    // checkLimits();
    // m_motor.set(ControlMode.PercentOutput, m_power);
  }
}
