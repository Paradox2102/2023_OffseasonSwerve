// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */

  public enum LEDMode {
    NULL,
    RAINBOW,
    READY,
    CONE,
    CUBE,
    PARTY

  }
  private AddressableLED m_led = new AddressableLED(0);
  private AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(30); // 62
  private int m_rainbowFirstPixelHue = 0;
  private int m_flashCounter = 0;
  private LEDMode m_mode = LEDMode.READY;
  private boolean m_isOff = true;

  public LEDSubsystem() {
    m_led.setLength(m_buffer.getLength());
    m_led.setData(m_buffer);
    m_led.start();
  }

  private void rainbow() {
    turnOn();
    // For every pixel
    for (var i = 0; i < m_buffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_buffer.getLength())) % 180;
      // Set the value
      m_buffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

  private void partyMode() {
    if (m_flashCounter < 50) {
      turnOn();
      rainbow();
    } else {
      m_led.stop();
    }
    m_flashCounter = m_flashCounter >= 100 ? 0 : m_flashCounter + 1;
  }

  private void ready() {
    if (m_flashCounter < 50) {
      turnOn();
      setGreen();
    } else {
      m_led.stop();
    }
    m_flashCounter = m_flashCounter >= 100 ? 0 : m_flashCounter + 1;
  }

  private void cone() {
    turnOn();
    for (int i = 0; i < m_buffer.getLength(); i++) {
      m_buffer.setRGB(i, 237, 234, 43);
    }
  }

  private void cube() {
    turnOn();
    for (int i = 0; i < m_buffer.getLength(); i++) {
      m_buffer.setRGB(i, 224, 38, 209);
    }
  }

  private void turnOn() {
    if (m_isOff) {
      m_isOff = false;
      m_led.start();
    }
  }

  private void setGreen() {
    turnOn();
    for (int i = 0; i < m_buffer.getLength(); i++) {
      m_buffer.setRGB(i, 66, 201, 48);
    }
  }

  public void setLED(LEDMode mode) {
    m_mode = mode;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // switch(m_mode) {
    //   case NULL:
    //     m_led.stop();
    //     m_isOff = true;
    //     break;
    //   case RAINBOW:
    //     rainbow();
    //     break;
    //   case READY:
    //     ready();
    //     break;
    //   case CONE:
    //     cone();
    //     break;
    //   case CUBE:
    //     cube();
    //     break;
    //   case PARTY:
    //     partyMode();
    //     break;
    // }

    for (var i = 0; i < m_buffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_buffer.setRGB(i, 255, 0, 0);
   }
   
    m_led.setData(m_buffer);

    // m_led.setData(m_buffer);
  }
}
