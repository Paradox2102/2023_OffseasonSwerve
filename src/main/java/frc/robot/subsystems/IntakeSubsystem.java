// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SignalLEDCommand;
import frc.robot.subsystems.LEDSubsystem.LEDMode;

public class IntakeSubsystem extends SubsystemBase {
  private double m_power = 0;
  private TalonFX m_motor = new TalonFX(Constants.k_intakeMotor);
  private final double k_stallSpeed = 0;
  private final double k_intakeMinPower = 0;
  private Timer m_stallTimer = new Timer();
  private LEDSubsystem m_LEDSubsytem;

  public enum IntakeType {INTAKE, OUTTAKE, STOP}


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(LEDSubsystem ledSubsystem) {
    m_stallTimer.reset();
    m_stallTimer.start();
    m_LEDSubsytem = ledSubsystem;
  }

  public void setPower() {
    m_power = 0;
  }

  public void intake() {
    m_power = Constants.k_isCubeMode ? Constants.CubeConstants.k_intakePower : Constants.ConeConstants.k_intakePower;
  }

  public void outtake() {
    m_power = Constants.k_isCubeMode ? Constants.CubeConstants.k_outtakePower : Constants.ConeConstants.k_outtakePower;
    new SignalLEDCommand(m_LEDSubsytem, LEDMode.READY);
  }

  public void stop() {
    m_power = 0;
  }

  // Lower power if game piece is acquired
  private void isIntakeStalled() {
    double speed = m_motor.getSelectedSensorVelocity();
    if (Math.abs(speed) < k_stallSpeed && Math.abs(m_power) > k_intakeMinPower && !Constants.k_hasGamePiece) {
      if (m_stallTimer.get() > .1) {
        m_power = k_intakeMinPower * Math.signum(m_power);
        Constants.k_hasGamePiece = true;
        new SignalLEDCommand(m_LEDSubsytem, LEDMode.READY);
      }
    } else {
      m_stallTimer.reset();
      Constants.k_hasGamePiece = false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    isIntakeStalled();
    m_motor.set(ControlMode.PercentOutput, m_power);
  }
}
