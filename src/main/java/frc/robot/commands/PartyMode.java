// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDMode;

public class PartyMode extends CommandBase {
  LEDSubsystem m_LEDSubsystem;
  DriveSubsystem m_driveSubsystem;
  /** Creates a new PartyMode. */
  public PartyMode(DriveSubsystem driveSubsystem, LEDSubsystem ledSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_LEDSubsystem = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveSubsystem.drive(0, 0, 1, true, false);
    m_LEDSubsystem.setLED(LEDMode.PARTY);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_LEDSubsystem.setLED(LEDMode.RAINBOW);
    m_driveSubsystem.setModuleStates(Constants.k_defaultState);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
