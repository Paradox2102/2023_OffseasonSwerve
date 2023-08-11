// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.ApriltagsCamera.Logger;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SignalLEDCommand extends InstantCommand {
  private LEDSubsystem m_subsystem;
  private LEDMode m_mode;
  public SignalLEDCommand(LEDSubsystem ledSubsystem, LEDMode mode) {
    m_subsystem = ledSubsystem;
    m_mode = mode;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("SignalLEDCommand", 0, "initialize");
    m_subsystem.setLED(m_mode);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
