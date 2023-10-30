// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  // private final AddressableLED m_leftLed = new AddressableLED(1);
  private final AddressableLED m_led = new AddressableLED(0);

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    // m_leftLed.setLength(Constants.k_LEDLength);
    m_led.setLength(Constants.k_LEDLength);
  }

  public void setData(AddressableLEDBuffer buffer) {
    // m_leftLed.setData(leftBuffer) ;
    m_led.setData(buffer);
  }

  public void start() {
    // m_leftLed.start();
    m_led.start();
  }

  public void end() {
    // m_leftLed.stop();
    m_led.stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
