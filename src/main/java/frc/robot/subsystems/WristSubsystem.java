// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
  private TalonFX m_motor = new TalonFX(Constants.k_wristMotor, "Default Name");
  private double k_deadzonePower = .015;
  private double k_f = 0;
  private double m_targetAngleDegrees = 0;
  private double m_power = 0;
  private boolean m_manualControl = false;
  private DigitalInput m_switch = new DigitalInput(6);

  private final double k_p = .025;
  private final double k_i = 0;
  private final double k_d = .00125;
  private PIDController m_PID = new PIDController(k_p, k_i, k_d);

  /** Creates a new WristSubsystem. */
  public WristSubsystem() {
    setBrakeMode(true);
    m_motor.setInverted(true);
    m_motor.setSelectedSensorPosition(0);
  }

  public void setAngleDegrees(double degree) {
    m_targetAngleDegrees = degree;
  }

  public void manualControl(boolean up, boolean manual) {
    m_manualControl = manual;
    m_targetAngleDegrees = getAngleDegrees() + 1 * (up ? -1 : 1);
    m_power = .15 * (up ? -1 : 1);
  }

  public void resetEncoder() {
    m_motor.setSelectedSensorPosition(0);
  }

  public void setPower(double power) {
    m_motor.set(ControlMode.PercentOutput, power);
  }

  public void setBrakeMode(boolean brake) {
    m_motor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
  }

  private double getAngleDegrees() {
    return m_motor.getSelectedSensorPosition() * Constants.k_wristTicksToDegrees;
  }

  private void checkLimits() {
    double angle = getAngleDegrees();
    boolean getSwitch = m_switch.get();
    if (m_power > 0 && angle >= Constants.k_maxAngleDegrees) {
      m_power = 0;
      m_motor.setSelectedSensorPosition(Constants.k_maxAngleDegrees);
    } else if (m_power < 0 && !getSwitch) {
      m_power = 0;
    }
    if (!getSwitch) {
      m_motor.setSelectedSensorPosition(Constants.k_minAngleDegrees);
    }
  }

  private void runP() {
    if (!m_manualControl) {
      m_power = m_PID.calculate(getAngleDegrees(), m_targetAngleDegrees) + k_deadzonePower;
    }
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    runP();
    checkLimits();
    System.out.println(m_power);
    m_motor.set(ControlMode.PercentOutput, m_power);
    SmartDashboard.putNumber("Wrist Pos", getAngleDegrees());
    SmartDashboard.putBoolean("Is Cube", Constants.k_isCubeMode);
    SmartDashboard.putBoolean("Wrist Switch", m_switch.get());
  }
}
