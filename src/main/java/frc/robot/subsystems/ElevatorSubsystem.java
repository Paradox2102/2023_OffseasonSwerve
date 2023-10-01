// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX m_motor = new TalonFX(Constants.k_elevatorMotor, "Default Name");
  private TalonFX m_follower = new TalonFX(Constants.k_elevatorFollower, "Default Name");
  private DigitalInput m_topSwitch = new DigitalInput(Constants.k_topSwitch);
  private DigitalInput m_bottomSwitch = new DigitalInput(Constants.k_bottomSwitch);
  private double m_power = 0;
  private double m_targetExtentInches = 0;
  private final double k_p = 1;
  private final double k_f = .003;
  private final double k_deadzone = .01;

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

  public void manualControl(double power) {
    m_power = power;
    m_targetExtentInches = getExtentInches();
  }

  public void setPower(double power) {
    m_motor.set(ControlMode.PercentOutput, power);
  }

  public void setBrakeMode(boolean brake) {
    m_motor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
  }

  private double getExtentInches() {
    return m_motor.getSelectedSensorPosition() * Constants.k_elevatorTicksToInches;
  }

  private void checkLimitSwitches() {
    if (m_power > 0) {
      if (m_topSwitch.get() || getExtentInches() >= Constants.k_maxExtentInches) {
        m_power = k_f;
        m_motor.setSelectedSensorPosition(Constants.k_maxExtentInches * Constants.k_elevatorInchesToTicks);
      } 
    } else if (m_power < 0) {
      if (m_bottomSwitch.get() || getExtentInches() <= Constants.k_minExtentInches) {
        m_power = 0;
        m_motor.setSelectedSensorPosition(Constants.k_minExtentInches * Constants.k_elevatorInchesToTicks);
      }
    }
  }

  private void runP() {
    double extentInches = getExtentInches();
    double distanceFromTarget = m_targetExtentInches - extentInches;
    double power = distanceFromTarget * k_p;
    // if (Math.abs(power) < .05) {
    //   m_power = .05 * Math.signum(power);
    // } else {
    //   m_power = power;
    // }
    // m_power = power;
    System.out.println(distanceFromTarget + " " +m_power);
    if (Math.abs(m_power) < k_deadzone) {
      m_power = k_f * extentInches;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    runP();
    // checkLimitSwitches();
    SmartDashboard.putNumber("Elevator Extent", m_motor.getSelectedSensorPosition());
    m_motor.set(ControlMode.PercentOutput, m_power);
  }
}
