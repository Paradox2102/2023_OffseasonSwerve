// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manual;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ManualElevatorCommand extends CommandBase {
  /** Creates a new ManualElevatorCommand. */
  ElevatorSubsystem m_subsystem;
  boolean m_up;
  public ManualElevatorCommand(ElevatorSubsystem elevatorSubsystem, boolean up) {
    m_subsystem = elevatorSubsystem;
    m_up = up;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.manualControl(m_up, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.manualControl(m_up, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}