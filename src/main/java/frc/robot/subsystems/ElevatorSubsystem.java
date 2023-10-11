// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX m_motor = new TalonFX(Constants.k_elevatorMotor, "Default Name");
  private TalonFX m_follower = new TalonFX(Constants.k_elevatorFollower, "Default Name");
  private DigitalInput m_midSwitch = new DigitalInput(Constants.k_midSwitch);
  private DigitalInput m_bottomSwitch = new DigitalInput(Constants.k_bottomSwitch);
  private DigitalInput m_topSwitch = new DigitalInput(Constants.k_topSwitch);
  private double m_power = 0;
  private double m_targetExtentInches = 0;
  private boolean m_manualControl = false;

  private final double k_p = .04;
  private final double k_i = 0;
  private final double k_d = .004;
  private PIDController m_PID = new PIDController(k_p, k_i, k_d);

  private final double k_FLow = .008;
  private final double k_FHigh = .01;
  private final double k_midHeightInches = 11;
  private final double k_maxDownPower = -.2;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_motor.setSelectedSensorPosition(0);
    m_follower.setSelectedSensorPosition(0);
    m_follower.follow(m_motor);
    m_follower.setInverted(false);
    m_motor.setInverted(true);
    m_motor.setSelectedSensorPosition(0);
    setBrakeMode(true);
  }

  public void setExtentInches(double targetExtentInches) {
    m_targetExtentInches = targetExtentInches;
  }

  public void manualControl(boolean up, boolean manual) {
    m_manualControl = manual;
    m_targetExtentInches = getExtentInches() + 1 * (up ? 1 : -1);
    m_power = .15 * (up ? 1 : -1) + getF();
  }

  private double getF() {
    if (getExtentInches() <= 0) {
      return 0;
    }
    return getExtentInches() > k_midHeightInches ? k_FHigh : k_FLow;
  }

  private void setPower(double power) {
    m_motor.set(ControlMode.PercentOutput, power);
  }

  public void setBrakeMode(boolean brake) {
    m_motor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
  }

  private double getExtentInches() {
    return m_motor.getSelectedSensorPosition() * Constants.k_elevatorTicksToInches;
  }

  private void checkLimitSwitches() {
    double extent = getExtentInches();
    if (m_power > 0) {
      if ((!m_midSwitch.get() && !m_topSwitch.get()) || extent >= Constants.k_maxExtentInches) {
        m_power = extent < k_midHeightInches ? k_FLow : k_FHigh;
        m_motor.setSelectedSensorPosition(Constants.k_maxExtentInches * Constants.k_elevatorInchesToTicks);
      } 
    } else if (m_power < 0) {
      if (!m_bottomSwitch.get() || extent <= Constants.k_minExtentInches) {
        m_power = 0;
        m_motor.setSelectedSensorPosition(Constants.k_minExtentInches * Constants.k_elevatorInchesToTicks);
      }
    }
  }

  private void runP() {
    double extentInches = getExtentInches();
    if (!m_manualControl) {
      m_power = m_PID.calculate(extentInches, m_targetExtentInches) + getF();
      m_power = m_power < k_maxDownPower ? k_maxDownPower : m_power;
    }
    SmartDashboard.putBoolean("Bottom Switch", m_bottomSwitch.get());
    SmartDashboard.putBoolean("Top Switch", m_topSwitch.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    runP();
    checkLimitSwitches();
    SmartDashboard.putNumber("Elevator Extent", m_motor.getSelectedSensorPosition() * Constants.k_elevatorTicksToInches);
    setPower(m_power);
  }
}
