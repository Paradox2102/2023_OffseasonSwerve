// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalanceCommand extends CommandBase {
  /** Creates a new AutoBalanceCommand. */
  DriveSubsystem m_subsystem;
  boolean m_isFinished = false;
  double k_p = .015;
  double m_previousPitch = 0;
  double m_futureRoll = 0;
  DoubleSupplier m_power = () -> 1;

  public AutoBalanceCommand(DriveSubsystem driveSubsystem) {
    m_subsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  public AutoBalanceCommand(DriveSubsystem driveSubsystem, DoubleSupplier power) {
    m_subsystem = driveSubsystem;
    m_power = power;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double heading = m_subsystem.getHeading();
    double currentPitch = m_subsystem.getRoll();
    double power = Math.abs(m_power.getAsDouble());

    // Robot's predicted pitch in half a second
    double futurePitch = 25 * (currentPitch - m_previousPitch) + currentPitch;

    // Facing forward
    if (Math.abs(heading) <= 50) {
      currentPitch = m_subsystem.getRoll();
    } 
    // Facing backward
    else if (Math.abs(heading) >= 130) {
      currentPitch = -m_subsystem.getRoll();
    }
    // Facing right
    else if (heading < 130 && heading > 50) {
      currentPitch = -m_subsystem.getPitch();
    }
    // Facing left
    else if (heading < -50 && heading > -130) {
      currentPitch = m_subsystem.getPitch();
    }

    futurePitch = (currentPitch - m_previousPitch) + currentPitch;

    if (Math.abs(futurePitch) < 2) {
      m_subsystem.setModuleStates(Constants.k_defaultState);
      m_isFinished = true;
    } else {
      m_subsystem.drive(k_p * futurePitch * power, 0, 0, true, false);
    }

    // Update previous to use on next call of execute
    m_previousPitch = currentPitch;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
