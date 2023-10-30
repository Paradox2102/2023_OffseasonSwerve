// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetLEDColorCommand extends InstantCommand {
  private final LEDSubsystem m_subsystem;
  private final AddressableLEDBuffer m_led = new AddressableLEDBuffer(Constants.k_LEDLength);  

  public SetLEDColorCommand(LEDSubsystem subsystem, Color leftColor, Color rightColor) {
    m_subsystem = subsystem;
    setLED(leftColor, rightColor);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  public void setLED(Color leftColor, Color rightColor) {
    for (int i = 0 ; i < Constants.k_LEDLength/2 ; i++) {
      m_led.setLED(i, rightColor);
    }

    for (int i = Constants.k_LEDLength/2 ; i < Constants.k_LEDLength ; i++) {
      m_led.setLED(i, leftColor);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setData(m_led);
    m_subsystem.start();
  }
}
