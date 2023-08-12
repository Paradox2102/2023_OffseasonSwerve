// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX m_motor = new TalonFX(Constants.k_elevatorMotor);
  private TalonFX m_follower = new TalonFX(Constants.k_elevatorFollower);
  private DigitalInput m_topSwitch = new DigitalInput(Constants.k_topSwitch);
  private DigitalInput m_bottomSwitch = new DigitalInput(Constants.k_bottomSwitch);
  private double m_power = 0;
  private double m_targetExtentInches = 0;
  private final double k_p = .01;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_motor.setSelectedSensorPosition(0);
    m_follower.setSelectedSensorPosition(0);
    m_follower.follow(m_motor);
    m_follower.setInverted(true);
    m_motor.setInverted(false);
  }

  public void setExtentInches(double targetExtentInches) {
    m_targetExtentInches = targetExtentInches;
  }

  public void manualExtent(double power) {
    m_power = power;
    m_targetExtentInches = getExtentInches();
  }

  private double getExtentInches() {
    return m_motor.getSelectedSensorPosition() * Constants.k_elevatorTicksToInches;
  }

  private void checkLimitSwitches() {
    if (m_power > 0) {
      if (m_topSwitch.get() || getExtentInches() >= Constants.k_maxExtentInches) {
        m_power = 0;
        m_motor.setSelectedSensorPosition(Constants.k_maxExtentInches);
      } 
    } else if (m_power < 0) {
      if (m_bottomSwitch.get() || getExtentInches() <= Constants.k_minExtentInches) {
        m_power = 0;
        m_motor.setSelectedSensorPosition(Constants.k_minExtentInches);
      }
    }
  }

  private void runP() {
    double extentInches = getExtentInches();
    double distanceFromTarget = m_targetExtentInches - extentInches;
    double power = distanceFromTarget * k_p;
    if (Math.abs(power) < .2) {
      m_power = .2 * Math.signum(power);
    } else {
      m_power = power;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    runP();
    checkLimitSwitches();
    m_motor.set(ControlMode.PercentOutput, m_power);
  }
}
