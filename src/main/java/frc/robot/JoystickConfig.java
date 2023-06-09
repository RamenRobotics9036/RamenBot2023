package frc.robot;

public class JoystickConfig {
  public int port;
  public int expectedAxisCount;
  public int expectedButtonCount;
  public int expectedPOVCount;
  public String expectedJoystickName;
  public int expectedJoystickType;

  // Constructor
  public JoystickConfig(int port,
      int expectedAxisCount,
      int expectedButtonCount,
      int expectedPOVCount,
      String expectedJoystickName,
      int expectedJoystickType) {

    this.port = port;
    this.expectedAxisCount = expectedAxisCount;
    this.expectedButtonCount = expectedButtonCount;
    this.expectedPOVCount = expectedPOVCount;
    this.expectedJoystickName = expectedJoystickName;
    this.expectedJoystickType = expectedJoystickType;
  }
}
