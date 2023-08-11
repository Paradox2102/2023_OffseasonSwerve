// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeCommand extends InstantCommand {
  IntakeSubsystem m_subsystem;
  String m_intakeType;
  public IntakeCommand(IntakeSubsystem intakeSubsystem, String intakeType) {
    m_subsystem = intakeSubsystem;
    m_intakeType = intakeType;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_intakeType.equals("intake")) {
      m_subsystem.intake();
    } else if (m_intakeType.equals("outtake")) {
      m_subsystem.outtake();
    } else if (m_intakeType.equals("stop")) {
      m_subsystem.stop();
    } else {
      System.out.println("Please use a valid intake argument (intake, outtake, stop)");
    }
  }
}
