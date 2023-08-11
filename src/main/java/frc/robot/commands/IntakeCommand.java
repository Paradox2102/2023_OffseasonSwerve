// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeType;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeCommand extends InstantCommand {
  IntakeSubsystem m_subsystem;
  IntakeType m_intakeType;
  public IntakeCommand(IntakeSubsystem intakeSubsystem, IntakeType intakeType) {
    m_subsystem = intakeSubsystem;
    m_intakeType = intakeType;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(m_intakeType) {
      case INTAKE:
        m_subsystem.intake();
        break;
      case OUTTAKE:
        m_subsystem.outtake();
        break;
      case STOP:
        m_subsystem.stop();
        break;
    }
  }
}
