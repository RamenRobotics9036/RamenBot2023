package frc.robot;

import java.time.Instant;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VerifyJoysticks {
  private class JoystickTest {
    private boolean m_lastSuccess;
    private final String m_joystickTestName;
    private final Supplier<String> m_testLambda;
    private final boolean m_stopRestOfTestsIfFailed;

    // Constructor
    public JoystickTest(String joystickTestName,
        boolean stopRestOfTestsIfFailed,
        Supplier<String> testLambda) {
      this.m_joystickTestName = joystickTestName;
      this.m_testLambda = testLambda;
      this.m_stopRestOfTestsIfFailed = stopRestOfTestsIfFailed;
      this.m_lastSuccess = true;
    }

    public boolean runTest(int port) {
      String errorMsg = this.m_testLambda.get();
      boolean newSuccess = errorMsg == null;

      // NOTE: We only print errors or successes when there's a change
      if (!newSuccess && m_lastSuccess) {
        System.out
            .println(String.format("Joystick test [%s] FAILED: %s", m_joystickTestName, errorMsg));
      }
      else if (newSuccess && !m_lastSuccess) {
        System.out.println(
            String.format("Joystick test [%s] passed on port=%d", m_joystickTestName, port));
      }

      m_lastSuccess = newSuccess;

      return newSuccess;
    }

    public boolean wasLastTestSuccessful() {
      return m_lastSuccess;
    }

    public String get_joystickTestName() {
      return m_joystickTestName;
    }

    public boolean is_stopRestOfTestsIfFailed() {
      return m_stopRestOfTestsIfFailed;
    }

    public void setTestAsSucceeded() {
      m_lastSuccess = true;
    }
  }

  // Member variables
  private DriverStationFunctions m_driverStationFunctions;
  private boolean m_lastResult;
  private boolean m_firstCall;
  private Instant m_recordedTime = Instant.now();
  private final JoystickConfig[] m_joystickConfigs;
  private JoystickTest[][] m_tests;
  private final int m_periodSeconds;

  // Constructor
  public VerifyJoysticks(JoystickConfig[] joystickConfigs,
      DriverStationFunctions driverStationFunctions,
      int periodSeconds) {
    // Check params
    if (joystickConfigs == null) {
      throw new IllegalArgumentException("joystickConfigs cannot be null");
    }

    if (driverStationFunctions == null) {
      throw new IllegalArgumentException("driverStationFunctions cannot be null");
    }

    m_periodSeconds = periodSeconds;

    m_driverStationFunctions = driverStationFunctions;
    m_joystickConfigs = joystickConfigs;
    m_tests = CreateAllJoystickTests(m_joystickConfigs);

    m_firstCall = true;
    m_lastResult = true;
  }

  public static JoystickConfig[] GetDefaultJoystickConfigs() {
    // Hardcoded expected joystick values
    return new JoystickConfig[] {
        new JoystickConfig(1, // port
            6, // expectedAxisCount
            16, // expectedButtonCount
            1, // expectedPOVCountt
            "Controller (Xbox One For Windows)", 1), // expectedJoystickType
        new JoystickConfig(0, 6, 16, 1, "Controller (Xbox One For Windows)", 1)
    };
  }

  private JoystickTest[][] CreateAllJoystickTests(JoystickConfig[] joystickConfigs) {
    JoystickTest[][] allTests = new JoystickTest[joystickConfigs.length][];

    for (int i = 0; i < joystickConfigs.length; i++) {
      allTests[i] = CreateInitializedJoystickTest(joystickConfigs[i].port,
          joystickConfigs[i].expectedAxisCount,
          joystickConfigs[i].expectedButtonCount,
          joystickConfigs[i].expectedPOVCount,
          joystickConfigs[i].expectedJoystickName,
          joystickConfigs[i].expectedJoystickType);
    }

    return allTests;
  }

  public boolean getIsJoystickConnected(int index) {
    if (index >= m_joystickConfigs.length) {
      throw new IllegalArgumentException("index is out of range");
    }
    return m_tests[index][0].wasLastTestSuccessful();
  }

  public boolean getIsAxisCountCorrect(int index) {
    if (index >= m_joystickConfigs.length) {
      throw new IllegalArgumentException("index is out of range");
    }
    return m_tests[index][1].wasLastTestSuccessful();
  }

  public boolean getIsTestButtonCountCorrect(int index) {
    if (index >= m_joystickConfigs.length) {
      throw new IllegalArgumentException("index is out of range");
    }
    return m_tests[index][2].wasLastTestSuccessful();
  }

  public boolean getIsPOVCountCorrect(int index) {
    if (index >= m_joystickConfigs.length) {
      throw new IllegalArgumentException("index is out of range");
    }
    return m_tests[index][3].wasLastTestSuccessful();
  }

  public boolean getIsJoystickNameCorrect(int index) {
    if (index >= m_joystickConfigs.length) {
      throw new IllegalArgumentException("index is out of range");
    }
    return m_tests[index][4].wasLastTestSuccessful();
  }

  public boolean getIsJoystickTypeCorrect(int index) {
    if (index >= m_joystickConfigs.length) {
      throw new IllegalArgumentException("index is out of range");
    }
    return m_tests[index][5].wasLastTestSuccessful();
  }

  public boolean getAreAllJoysticksHealth() {
    return m_lastResult;
  }

  private JoystickTest[] CreateInitializedJoystickTest(int port,
      int expectedAxisCount,
      int expectedButtonCount,
      int expectedPOVCount,
      String expectedJoystickName,
      int expectedJoystickType) {

    JoystickTest[] tests = new JoystickTest[] {
        new JoystickTest("isJoystickConnected", true, () -> TestIsJoystickConnected(port)),
        new JoystickTest("TestAxisCount", false, () -> TestAxisCount(port, expectedAxisCount)),
        new JoystickTest("TestButtonCount", false,
            () -> TestButtonCount(port, expectedButtonCount)),
        new JoystickTest("TestPOVCount", false, () -> TestPOVCount(port, expectedPOVCount)),
        new JoystickTest("TestJoystickName", false,
            () -> TestJoystickName(port, expectedJoystickName)),
        new JoystickTest("TestJoystickType", false,
            () -> TestJoystickType(port, expectedJoystickType))
    };

    return tests;
  }

  public boolean VerifyJoysticksPeriodically() {
    Instant currentTime = Instant.now();
    boolean AllSuccess = true;

    if (m_firstCall
        || (currentTime.getEpochSecond() - m_recordedTime.getEpochSecond()) >= m_periodSeconds) {

      for (int i = 0; i < m_joystickConfigs.length; i++) {
        if (!VerifySingleJoystick(m_joystickConfigs[i].port,
            m_joystickConfigs[i].expectedAxisCount,
            m_joystickConfigs[i].expectedButtonCount,
            m_joystickConfigs[i].expectedPOVCount,
            m_joystickConfigs[i].expectedJoystickName,
            m_joystickConfigs[i].expectedJoystickType,
            m_tests[i])) {

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

  private String TestIsJoystickConnected(int port) {
    if (!m_driverStationFunctions.isJoystickConnected(port)) {
      return "No joystick plugged into port " + port;
    }

    return null;
  }

  private String TestAxisCount(int port, int expectedAxisCount) {
    int actualAxisCount = m_driverStationFunctions.getStickAxisCount(port);
    if (expectedAxisCount != actualAxisCount) {
      return String.format("Port=%d, Expected AxisCount=%d, Actual AxisCount=%d, ",
          port,
          expectedAxisCount,
          actualAxisCount);
    }

    return null;
  }

  private String TestButtonCount(int port, int expectedButtonCount) {
    int actualButtonCount = m_driverStationFunctions.getStickButtonCount(port);
    if (expectedButtonCount != actualButtonCount) {
      return String.format("Port=%d, Expected ButtonCount=%d, Actual ButtonCount=%d",
          port,
          expectedButtonCount,
          actualButtonCount);
    }

    return null;
  }

  private String TestPOVCount(int port, int expectedPOVCount) {
    int actualPovCount = m_driverStationFunctions.getStickPOVCount(port);
    if (expectedPOVCount != actualPovCount) {
      return String.format("Port=%d, Expected PovCount=%d, Actual PovCount=%d",
          port,
          expectedPOVCount,
          actualPovCount);
    }

    return null;
  }

  private String TestJoystickName(int port, String expectedJoystickName) {
    String actualJoystickName = m_driverStationFunctions.getJoystickName(port);
    if (!expectedJoystickName.equals(actualJoystickName)) {
      return String.format("Port=%d, Expected JoystickName=%s, Actual JoystickName=%s",
          port,
          expectedJoystickName,
          actualJoystickName);
    }

    return null;
  }

  private String TestJoystickType(int port, int expectedJoystickType) {
    int actualJoystickType = m_driverStationFunctions.getJoystickType(port);
    if (expectedJoystickType != actualJoystickType) {
      return String.format("Port=%d, Expected JoystickType=%d, Actual JoystickType=%d",
          port,
          expectedJoystickType,
          actualJoystickType);
    }

    return null;
  }

  private static void ResetAllTestsToPassedExceptJoystickUnplugged(JoystickTest[] tests) {
    for (int i = 0; i < tests.length; i++) {
      if (!tests[i].is_stopRestOfTestsIfFailed()) {
        tests[i].setTestAsSucceeded();
      }
    }
  }

  private boolean VerifySingleJoystick(int port,
      int expectedAxisCount,
      int expectedButtonCount,
      int expectedPOVCount,
      String expectedJoystickName,
      int expectedJoystickType,
      JoystickTest[] tests) {

    boolean result = true;

    // Loop through the tests, and run each
    for (int i = 0; i < tests.length; i++) {
      if (!tests[i].runTest(port)) {
        result = false;

        if (tests[i].is_stopRestOfTestsIfFailed()) {
          // When the joystick is UNPLUGGED, we reset all the OTHER joystick test results
          // to PASSED. That way, when a new joystick is plugged in again,
          // all those other tests will rerun even if they were already failed on the
          // previously plugged-in Joystick. That will cause this system to print ALL
          // the unexpected configurations on the NEW joystick
          ResetAllTestsToPassedExceptJoystickUnplugged(tests);
          break;
        }
      }
    }

    return result;
  }
}
