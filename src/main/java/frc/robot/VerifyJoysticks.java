package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import java.time.Instant;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VerifyJoysticks {
  public class JoystickConfig {
      int port;
      int expectedAxisCount;
      int expectedButtonCount;
      int expectedPOVCount;
      String expectedJoystickName;
      int expectedJoystickType;

      // Constructor
      public JoystickConfig(
          int port,
          int expectedAxisCount,
          int expectedButtonCount,
          int expectedPOVCount,
          String expectedJoystickName,
          int expectedJoystickType
      ) {
          this.port = port;
          this.expectedAxisCount = expectedAxisCount;
          this.expectedButtonCount = expectedButtonCount;
          this.expectedPOVCount = expectedPOVCount;
          this.expectedJoystickName = expectedJoystickName;
          this.expectedJoystickType = expectedJoystickType;
      }
  }

  private boolean m_lastResult;
  private boolean m_firstCall;
  private Instant m_recordedTime;

  private JoystickConfig[] m_joystickConfigs = new JoystickConfig[] {
      new JoystickConfig(
        1,    // port
        6,    // expectedAxisCount
        10,    // expectedButtonCount
        1,    // expectedPOVCountt
        "Xbox Controller",
        21)   // expectedJoystickType
  };

  // Constructor
  public VerifyJoysticks() {
    m_firstCall = true;
  }

  public boolean VerifyJoysticksPeriodically() {
    Instant currentTime = Instant.now();
    int periodSeconds = 10;
    boolean AllSuccess = true;

    if (m_firstCall || (currentTime.getEpochSecond() - m_recordedTime.getEpochSecond()) >= periodSeconds) {

      for (int i = 0; i < m_joystickConfigs.length; i++) {
         if (!VerifySingleJoystick(
          m_joystickConfigs[i].port,
          m_joystickConfigs[i].expectedAxisCount,
          m_joystickConfigs[i].expectedButtonCount,
          m_joystickConfigs[i].expectedPOVCount,
          m_joystickConfigs[i].expectedJoystickName,
          m_joystickConfigs[i].expectedJoystickType)) {

            AllSuccess = false;
          }
      }

      m_lastResult = AllSuccess;
      m_recordedTime = currentTime;
      m_firstCall = false;

      // We only update the dashboard every few seconds
      UpdateDashboard();
    }

    return m_lastResult;
  }

  private void UpdateDashboard() {
    SmartDashboard.putBoolean("Joystick health", m_lastResult);
  }

  private boolean VerifySingleJoystick(
    int port,
    int expectedAxisCount,
    int expectedButtonCount,
    int expectedPOVCount,
    String expectedJoystickName,
    int expectedJoystickType) {

    boolean result = true;

    if (!DriverStation.isJoystickConnected(port)) {
      System.out.println("No joystick plugged into port " + port);
      return false;
    }

    int actualAxisCount = DriverStation.getStickAxisCount(port);
    if (expectedAxisCount != actualAxisCount) {
      System.out.println(String.format("Joystick port #%d: Expected AxisCount=%d, Actual AxisCount=%d", port, expectedAxisCount, actualAxisCount));
      result = false;
    }

    int actualButtonCount = DriverStation.getStickButtonCount(port);
    if (expectedButtonCount != actualButtonCount) {
      System.out.println(String.format("Joystick port #%d: Expected ButtonCount=%d, Actual ButtonCount=%d", port, expectedButtonCount, actualButtonCount));
      result = false;
    }

    int actualPovCount = DriverStation.getStickPOVCount(port);
    if (expectedPOVCount != actualPovCount) {
      System.out.println(String.format("Joystick port #%d: Expected PovCount=%d, Actual PovCount=%d", port, expectedPOVCount, actualPovCount));
      result = false;
    }

    String actualJoystickName = DriverStation.getJoystickName(port);
    if (!expectedJoystickName.equals(actualJoystickName)) {
      System.out.println(String.format("Joystick port #%d: Expected JoystickName=%s, Actual JoystickName=%s", port, expectedJoystickName, actualJoystickName));
      result = false;
    }

    int actualJoystickType = DriverStation.getJoystickType(port);
    if (expectedJoystickType != actualJoystickType) {
      System.out.println(String.format("Joystick port #%d: Expected JoystickType=%d, Actual JoystickType=%d", port, expectedJoystickType, actualJoystickType));
      result = false;
    }

    return result;
  }
}

