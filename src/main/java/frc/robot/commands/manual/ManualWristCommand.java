// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manual;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class ManualWristCommand extends CommandBase {
  /** Creates a new ManualWristCommand. */
  WristSubsystem m_subsystem;
  DoubleSupplier m_up;
  public ManualWristCommand(WristSubsystem wristSubsystem, DoubleSupplier up) {
    m_subsystem = wristSubsystem;
    m_up = up;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double up = -m_up.getAsDouble();
    if (up == 0) {
      m_subsystem.manualControl(true, false);
    } else {
      m_subsystem.manualControl(up < 0, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.manualControl(true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
