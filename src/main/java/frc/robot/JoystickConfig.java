package frc.robot;

public class JoystickConfig {
  int port;
  int expectedAxisCount;
  int expectedButtonCount;
  int expectedPOVCount;
  String expectedJoystickName;
  int expectedJoystickType;

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
