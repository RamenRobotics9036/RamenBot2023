package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * Updates the LED lights on the robot.
 */
public class LedLights {
  private AddressableLED m_ledLight = new AddressableLED(
      Constants.OperatorConstants.kLEDLightsChannel);
  private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(
      Constants.OperatorConstants.kLEDLightsLength);

  private int m_ledLoop;
  private int m_ledR;
  private int m_ledG;
  private int m_ledB;
  private int m_ledHue;

  /**
   * Constructor.
   */
  public LedLights() {
    m_ledLoop = 0;
    m_ledR = 0;
    m_ledG = 0;
    m_ledB = 0;
    m_ledHue = 0;
    m_ledLight.setLength(m_ledBuffer.getLength());
  }

  /**
   * Updates the LED lights on the robot.
   */
  public void updateLeds() {
    if (0 == m_ledR && 0 == m_ledG && 0 == m_ledB) {
      for (var i = 0; i < m_ledBuffer.getLength() / 2; i++) {
        var hue = (m_ledHue + (i * 180 / (m_ledBuffer.getLength() / 2))) % 180;
        m_ledBuffer.setHSV(i, hue, 255, 128);
        m_ledBuffer.setHSV(m_ledBuffer.getLength() - i - 1, hue, 255, 128);
      }
      m_ledHue += 2;
      m_ledHue %= 180;

    }
    else {
      for (var i = 0; i < m_ledBuffer.getLength() / 2; i++) {
        if (i == m_ledLoop) {
          m_ledBuffer.setRGB(i, 0, 0, 0);
          m_ledBuffer.setRGB(m_ledBuffer.getLength() - i - 1, 0, 0, 0);
        }
        else {
          m_ledBuffer.setRGB(i, m_ledG, m_ledR, m_ledB);
          m_ledBuffer.setRGB(m_ledBuffer.getLength() - i - 1, m_ledG, m_ledR, m_ledB);
        }
      }
    }
    m_ledLight.setData(m_ledBuffer);
    m_ledLight.start();

    m_ledLoop -= 1;
    if (m_ledLoop < 0) {
      m_ledLoop = m_ledBuffer.getLength() / 2;
    }
  }

  /**
   * Resets the LED lights. Usually called when we enter teleop or auto modes.
   */
  public void resetLeds() {
    m_ledR = 0;
    m_ledG = 0;
    m_ledB = 0;
  }

  /**
   * Sets the LED lights to yellow.
   */
  public void setLedsYellow() {
    m_ledR = 255;
    m_ledG = 255;
    m_ledB = 0;
  }

  /**
   * Sets the LED lights to magenta.
   */
  public void setLedsMagenta() {
    m_ledR = 255;
    m_ledG = 0;
    m_ledB = 255;
  }
}
