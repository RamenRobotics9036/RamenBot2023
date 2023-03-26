package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import java.time.Instant;

public class VerifyJoysticks {
  private static final int EXPECTED_AXIS_COUNT = 6;
  private static final int EXPECTED_BUTTON_COUNT = 12;
  private static final int EXPECTED_POV_COUNT = 1;
  private boolean m_lastResult;
  private boolean m_firstCall;
  private Instant m_recordedTime;

  // Constructor
  public VerifyJoysticks() {
    m_firstCall = true;
  }

  public boolean VerifyJoysticksPeriodically() {
    Instant currentTime = Instant.now();
    int periodSeconds = 10;

    if (m_firstCall || (currentTime.getEpochSecond() - m_recordedTime.getEpochSecond()) >= periodSeconds) {
      m_lastResult = VerifySingleJoystick(0);
      m_recordedTime = currentTime;
      m_firstCall = false;
    }

    return m_lastResult;
  }

  private boolean VerifySingleJoystick(int port) {

    boolean isJoystick = DriverStation.isJoystickConnected(port);
    if (!isJoystick) {
      System.out.println("No joystick plugged into port " + port);
      return false;
    }

    int axisCount = DriverStation.getStickAxisCount(port);
    int buttonCount = DriverStation.getStickButtonCount(port);
    int povCount = DriverStation.getStickPOVCount(port);

    System.out.println("Joystick Name: " + DriverStation.getJoystickName(port));
    System.out.println("Joystick Type: " + DriverStation.getJoystickType(port));

    for (int i = 0; i < axisCount; i++) {
      System.out.println("Axis " + i + " Type: " + DriverStation.getJoystickAxisType(port, i));
    }

    boolean isExpectedJoystick =
            axisCount == EXPECTED_AXIS_COUNT &&
            buttonCount == EXPECTED_BUTTON_COUNT &&
            povCount == EXPECTED_POV_COUNT;

    if (!isExpectedJoystick) {
      System.out.println("Joystick verification result: FAILURE");
    }

    return isExpectedJoystick;
  }
}

