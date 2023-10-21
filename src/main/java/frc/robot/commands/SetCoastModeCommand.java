// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetCoastModeCommand extends InstantCommand {
  WristSubsystem m_wristSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  BooleanSupplier m_brake;
  public SetCoastModeCommand(WristSubsystem wristSubsystem, ElevatorSubsystem elevatorSubsystem, BooleanSupplier brake) {
    m_wristSubsystem = wristSubsystem;
    m_elevatorSubsystem = elevatorSubsystem;
    m_brake = brake;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wristSubsystem.setBrakeMode(!m_wristSubsystem.getBrake());
    m_elevatorSubsystem.setBrakeMode(!m_elevatorSubsystem.getBrake());
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
