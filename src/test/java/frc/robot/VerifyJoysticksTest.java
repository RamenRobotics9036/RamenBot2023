package frc.robot;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;
import static org.junit.jupiter.api.Assertions.assertThrows;

public class VerifyJoysticksTest {
  private static JoystickConfig[] GetTestJoystickConfigs() {
    return new JoystickConfig[] {
        new JoystickConfig(1, // port
            6, // expectedAxisCount
            16, // expectedButtonCount
            1, // expectedPOVCountt
            "Controller (Xbox One For Windows)", 1) // expectedJoystickType
    };
  }

  private static JoystickConfig[] GetTwoTestJoysticksConfigs() {
    return new JoystickConfig[] {
        new JoystickConfig(1, // port
            6, // expectedAxisCount
            16, // expectedButtonCount
            1, // expectedPOVCountt
            "Controller (Xbox One For Windows)", 1), // expectedJoystickType
        new JoystickConfig(2, // port
            5, // expectedAxisCount
            4, // expectedButtonCount
            8, // expectedPOVCountt
            "Joystick 2", 20) // expectedJoystickType
    };
  }

  @BeforeEach
  public void setUp() {
  }

  @Test
  public void CreateVerifyJoysticksShouldSucceed() {
    VerifyJoysticks verifyTemp = new VerifyJoysticks(GetTestJoystickConfigs(),
        new DriverStationFunctions(), 0);

    assertTrue(verifyTemp != null);
  }

  @Test
  public void CreateVerifyJoysticksWithNullJoystickConfigsShouldFail() {
    assertThrows(IllegalArgumentException.class,
        () -> new VerifyJoysticks(null, new DriverStationFunctions(), 0));
  }

  @Test
  public void CreateVerifyJoysticksWithNullDriverStationFunctionsShouldFail() {
    assertThrows(IllegalArgumentException.class,
        () -> new VerifyJoysticks(GetTestJoystickConfigs(), null, 0));
  }

  @Test
  void JoystickUnpluggedShouldShowAsUnplugged() {
    int port = 1;

    DriverStationFunctions mockDriverStationFunctions = mock(DriverStationFunctions.class);
    when(mockDriverStationFunctions.isJoystickConnected(port)).thenReturn(false);

    VerifyJoysticks verifyTemp = new VerifyJoysticks(GetTestJoystickConfigs(),
        mockDriverStationFunctions, 0);

    assertTrue(verifyTemp != null);
    assertTrue(verifyTemp.getAreAllJoysticksHealth());

    verifyTemp.VerifyJoysticksPeriodically();
    assertTrue(!verifyTemp.getAreAllJoysticksHealth());
    assertTrue(!verifyTemp.getIsJoystickConnected(0));
  }

  @Test
  void JoystickPluggedInShouldShowAsPluggedIn() {
    int port = 1;

    DriverStationFunctions mockDriverStationFunctions = mock(DriverStationFunctions.class);

    when(mockDriverStationFunctions.isJoystickConnected(port)).thenReturn(true);
    when(mockDriverStationFunctions.getStickAxisCount(port)).thenReturn(6);
    when(mockDriverStationFunctions.getStickButtonCount(port)).thenReturn(16);
    when(mockDriverStationFunctions.getStickPOVCount(port)).thenReturn(1);
    when(mockDriverStationFunctions.getJoystickName(port))
        .thenReturn("Controller (Xbox One For Windows)");
    when(mockDriverStationFunctions.getJoystickType(port)).thenReturn(1);

    VerifyJoysticks verifyTemp = new VerifyJoysticks(GetTestJoystickConfigs(),
        mockDriverStationFunctions, 0);

    assertTrue(verifyTemp != null);
    assertTrue(verifyTemp.getAreAllJoysticksHealth());

    verifyTemp.VerifyJoysticksPeriodically();
    assertTrue(verifyTemp.getAreAllJoysticksHealth());
    assertTrue(verifyTemp.getIsJoystickConnected(0));
  }

  @Test
  void JoystickWrongAxisCountShowsAsBroken() {
    int port = 1;

    DriverStationFunctions mockDriverStationFunctions = mock(DriverStationFunctions.class);

    when(mockDriverStationFunctions.isJoystickConnected(port)).thenReturn(true);
    when(mockDriverStationFunctions.getStickAxisCount(port)).thenReturn(7); // this value is changed
    when(mockDriverStationFunctions.getStickButtonCount(port)).thenReturn(16);
    when(mockDriverStationFunctions.getStickPOVCount(port)).thenReturn(1);
    when(mockDriverStationFunctions.getJoystickName(port))
        .thenReturn("Controller (Xbox One For Windows)");
    when(mockDriverStationFunctions.getJoystickType(port)).thenReturn(1);

    VerifyJoysticks verifyTemp = new VerifyJoysticks(GetTestJoystickConfigs(),
        mockDriverStationFunctions, 0);

    assertTrue(verifyTemp != null);
    assertTrue(verifyTemp.getAreAllJoysticksHealth());

    verifyTemp.VerifyJoysticksPeriodically();
    assertTrue(!verifyTemp.getAreAllJoysticksHealth());
    assertTrue(!verifyTemp.getIsAxisCountCorrect(0));
  }

  @Test
  void JoystickWrongButtonCountShowsAsBroken() {
    int port = 1;

    DriverStationFunctions mockDriverStationFunctions = mock(DriverStationFunctions.class);

    when(mockDriverStationFunctions.isJoystickConnected(port)).thenReturn(true);
    when(mockDriverStationFunctions.getStickAxisCount(port)).thenReturn(6);
    when(mockDriverStationFunctions.getStickButtonCount(port)).thenReturn(17); // this value is
                                                                               // changed
    when(mockDriverStationFunctions.getStickPOVCount(port)).thenReturn(1);
    when(mockDriverStationFunctions.getJoystickName(port))
        .thenReturn("Controller (Xbox One For Windows)");
    when(mockDriverStationFunctions.getJoystickType(port)).thenReturn(1);

    VerifyJoysticks verifyTemp = new VerifyJoysticks(GetTestJoystickConfigs(),
        mockDriverStationFunctions, 0);

    assertTrue(verifyTemp != null);
    assertTrue(verifyTemp.getAreAllJoysticksHealth());

    verifyTemp.VerifyJoysticksPeriodically();
    assertTrue(!verifyTemp.getAreAllJoysticksHealth());
    assertTrue(!verifyTemp.getIsTestButtonCountCorrect(0));
  }

  @Test
  void JoystickWrongPOVCountShowsAsBroken() {
    int port = 1;

    DriverStationFunctions mockDriverStationFunctions = mock(DriverStationFunctions.class);

    when(mockDriverStationFunctions.isJoystickConnected(port)).thenReturn(true);
    when(mockDriverStationFunctions.getStickAxisCount(port)).thenReturn(6);
    when(mockDriverStationFunctions.getStickButtonCount(port)).thenReturn(16);
    when(mockDriverStationFunctions.getStickPOVCount(port)).thenReturn(2); // this value is changed
    when(mockDriverStationFunctions.getJoystickName(port))
        .thenReturn("Controller (Xbox One For Windows)");
    when(mockDriverStationFunctions.getJoystickType(port)).thenReturn(1);

    VerifyJoysticks verifyTemp = new VerifyJoysticks(GetTestJoystickConfigs(),
        mockDriverStationFunctions, 0);

    assertTrue(verifyTemp != null);
    assertTrue(verifyTemp.getAreAllJoysticksHealth());

    verifyTemp.VerifyJoysticksPeriodically();
    assertTrue(!verifyTemp.getAreAllJoysticksHealth());
    assertTrue(!verifyTemp.getIsPOVCountCorrect(0));
  }

  @Test
  void JoystickWrongJoystickNameShowsAsBroken() {
    int port = 1;

    DriverStationFunctions mockDriverStationFunctions = mock(DriverStationFunctions.class);

    when(mockDriverStationFunctions.isJoystickConnected(port)).thenReturn(true);
    when(mockDriverStationFunctions.getStickAxisCount(port)).thenReturn(6);
    when(mockDriverStationFunctions.getStickButtonCount(port)).thenReturn(16);
    when(mockDriverStationFunctions.getStickPOVCount(port)).thenReturn(1);
    when(mockDriverStationFunctions.getJoystickName(port)).thenReturn("Controller (For Windows)"); // this
                                                                                                   // value
                                                                                                   // is
                                                                                                   // changed
    when(mockDriverStationFunctions.getJoystickType(port)).thenReturn(1);

    VerifyJoysticks verifyTemp = new VerifyJoysticks(GetTestJoystickConfigs(),
        mockDriverStationFunctions, 0);

    assertTrue(verifyTemp != null);
    assertTrue(verifyTemp.getAreAllJoysticksHealth());

    verifyTemp.VerifyJoysticksPeriodically();
    assertTrue(!verifyTemp.getAreAllJoysticksHealth());
    assertTrue(!verifyTemp.getIsJoystickNameCorrect(0));
  }

  @Test
  void JoystickWrongJoystickTypeShowsAsBroken() {
    int port = 1;

    DriverStationFunctions mockDriverStationFunctions = mock(DriverStationFunctions.class);

    when(mockDriverStationFunctions.isJoystickConnected(port)).thenReturn(true);
    when(mockDriverStationFunctions.getStickAxisCount(port)).thenReturn(6);
    when(mockDriverStationFunctions.getStickButtonCount(port)).thenReturn(16);
    when(mockDriverStationFunctions.getStickPOVCount(port)).thenReturn(1);
    when(mockDriverStationFunctions.getJoystickName(port))
        .thenReturn("Controller (Xbox One For Windows)");
    when(mockDriverStationFunctions.getJoystickType(port)).thenReturn(2); // this value is changed

    VerifyJoysticks verifyTemp = new VerifyJoysticks(GetTestJoystickConfigs(),
        mockDriverStationFunctions, 0);

    assertTrue(verifyTemp != null);
    assertTrue(verifyTemp.getAreAllJoysticksHealth());

    verifyTemp.VerifyJoysticksPeriodically();
    assertTrue(!verifyTemp.getAreAllJoysticksHealth());
    assertTrue(!verifyTemp.getIsJoystickTypeCorrect(0));

    // For good measure, lets verify that the other settings are healthy
    assertTrue(verifyTemp.getIsJoystickConnected(0));
    assertTrue(verifyTemp.getIsAxisCountCorrect(0));
    assertTrue(verifyTemp.getIsTestButtonCountCorrect(0));
    assertTrue(verifyTemp.getIsPOVCountCorrect(0));
    assertTrue(verifyTemp.getIsJoystickNameCorrect(0));
  }

  @Test
  void TestingTwoJoysticksShouldWork() {
    DriverStationFunctions mockDriverStationFunctions = mock(DriverStationFunctions.class);

    // Joystick on port 1
    when(mockDriverStationFunctions.isJoystickConnected(1)).thenReturn(true);
    when(mockDriverStationFunctions.getStickAxisCount(1)).thenReturn(6);
    when(mockDriverStationFunctions.getStickButtonCount(1)).thenReturn(16);
    when(mockDriverStationFunctions.getStickPOVCount(1)).thenReturn(1);
    when(mockDriverStationFunctions.getJoystickName(1))
        .thenReturn("Controller (Xbox One For Windows)");
    when(mockDriverStationFunctions.getJoystickType(1)).thenReturn(1);

    // Joystick on port 2
    when(mockDriverStationFunctions.isJoystickConnected(2)).thenReturn(true);
    when(mockDriverStationFunctions.getStickAxisCount(2)).thenReturn(5);
    when(mockDriverStationFunctions.getStickButtonCount(2)).thenReturn(4);
    when(mockDriverStationFunctions.getStickPOVCount(2)).thenReturn(8);
    when(mockDriverStationFunctions.getJoystickName(2)).thenReturn("Joystick 2");
    when(mockDriverStationFunctions.getJoystickType(2)).thenReturn(20);

    VerifyJoysticks verifyTemp = new VerifyJoysticks(GetTwoTestJoysticksConfigs(),
        mockDriverStationFunctions, 0);

    assertTrue(verifyTemp != null);
    assertTrue(verifyTemp.getAreAllJoysticksHealth());

    verifyTemp.VerifyJoysticksPeriodically();
    assertTrue(verifyTemp.getAreAllJoysticksHealth());

    // Verify joystick on port 1
    assertTrue(verifyTemp.getIsJoystickConnected(0));
    assertTrue(verifyTemp.getIsAxisCountCorrect(0));
    assertTrue(verifyTemp.getIsTestButtonCountCorrect(0));
    assertTrue(verifyTemp.getIsPOVCountCorrect(0));
    assertTrue(verifyTemp.getIsJoystickNameCorrect(0));
    assertTrue(verifyTemp.getIsJoystickTypeCorrect(0));

    // Verify joystick on port 2
    assertTrue(verifyTemp.getIsJoystickConnected(1));
    assertTrue(verifyTemp.getIsAxisCountCorrect(1));
    assertTrue(verifyTemp.getIsTestButtonCountCorrect(1));
    assertTrue(verifyTemp.getIsPOVCountCorrect(1));
    assertTrue(verifyTemp.getIsJoystickNameCorrect(1));
    assertTrue(verifyTemp.getIsJoystickTypeCorrect(1));
  }

  @Test
  void WhenJoystickGetsUnpluggedRestOfParamsShouldShowAsHealthy() {
    int port = 1;

    DriverStationFunctions mockDriverStationFunctions = mock(DriverStationFunctions.class);

    when(mockDriverStationFunctions.isJoystickConnected(port)).thenReturn(true).thenReturn(false);
    when(mockDriverStationFunctions.getStickAxisCount(port)).thenReturn(6);
    when(mockDriverStationFunctions.getStickButtonCount(port)).thenReturn(16);
    when(mockDriverStationFunctions.getStickPOVCount(port)).thenReturn(1);
    when(mockDriverStationFunctions.getJoystickName(port))
        .thenReturn("Controller (Xbox One For Windows)");
    when(mockDriverStationFunctions.getJoystickType(port)).thenReturn(2); // this value is changed

    VerifyJoysticks verifyTemp = new VerifyJoysticks(GetTestJoystickConfigs(),
        mockDriverStationFunctions, 0);

    assertTrue(verifyTemp != null);
    assertTrue(verifyTemp.getAreAllJoysticksHealth());

    verifyTemp.VerifyJoysticksPeriodically();

    // On the first round, JoystickType should be unhealthy
    assertTrue(!verifyTemp.getAreAllJoysticksHealth());
    assertTrue(!verifyTemp.getIsJoystickTypeCorrect(0));
    assertTrue(verifyTemp.getIsJoystickConnected(0));
    assertTrue(verifyTemp.getIsAxisCountCorrect(0));
    assertTrue(verifyTemp.getIsTestButtonCountCorrect(0));
    assertTrue(verifyTemp.getIsPOVCountCorrect(0));
    assertTrue(verifyTemp.getIsJoystickNameCorrect(0));

    verifyTemp.VerifyJoysticksPeriodically();

    // Now, the joystick was unplugged. All other params should be healthy
    assertTrue(!verifyTemp.getAreAllJoysticksHealth());
    assertTrue(!verifyTemp.getIsJoystickConnected(0));
    assertTrue(verifyTemp.getIsAxisCountCorrect(0));
    assertTrue(verifyTemp.getIsTestButtonCountCorrect(0));
    assertTrue(verifyTemp.getIsPOVCountCorrect(0));
    assertTrue(verifyTemp.getIsJoystickNameCorrect(0));
    assertTrue(verifyTemp.getIsJoystickTypeCorrect(0));
  }

  @Test
  void WhenElapsedTimerUsedThenQuicklyRequeringShouldGiveOldResult() {
    int port = 1;
    int timerSeconds = 60 * 60 * 24;

    DriverStationFunctions mockDriverStationFunctions = mock(DriverStationFunctions.class);

    when(mockDriverStationFunctions.isJoystickConnected(port)).thenReturn(true).thenReturn(false);
    when(mockDriverStationFunctions.getStickAxisCount(port)).thenReturn(6);
    when(mockDriverStationFunctions.getStickButtonCount(port)).thenReturn(16);
    when(mockDriverStationFunctions.getStickPOVCount(port)).thenReturn(1);
    when(mockDriverStationFunctions.getJoystickName(port))
        .thenReturn("Controller (Xbox One For Windows)");
    when(mockDriverStationFunctions.getJoystickType(port)).thenReturn(2); // this value is changed

    VerifyJoysticks verifyTemp = new VerifyJoysticks(GetTestJoystickConfigs(),
        mockDriverStationFunctions, timerSeconds);

    assertTrue(verifyTemp != null);
    assertTrue(verifyTemp.getAreAllJoysticksHealth());

    verifyTemp.VerifyJoysticksPeriodically();

    // On the first round, JoystickType should be unhealthy
    assertTrue(!verifyTemp.getAreAllJoysticksHealth());
    assertTrue(!verifyTemp.getIsJoystickTypeCorrect(0));
    assertTrue(verifyTemp.getIsJoystickConnected(0));
    assertTrue(verifyTemp.getIsAxisCountCorrect(0));
    assertTrue(verifyTemp.getIsTestButtonCountCorrect(0));
    assertTrue(verifyTemp.getIsPOVCountCorrect(0));
    assertTrue(verifyTemp.getIsJoystickNameCorrect(0));

    verifyTemp.VerifyJoysticksPeriodically();

    // Since timer is set to 24 hours, status on second query shouldnt change
    assertTrue(!verifyTemp.getAreAllJoysticksHealth());
    assertTrue(!verifyTemp.getIsJoystickTypeCorrect(0));
    assertTrue(verifyTemp.getIsJoystickConnected(0));
    assertTrue(verifyTemp.getIsAxisCountCorrect(0));
    assertTrue(verifyTemp.getIsTestButtonCountCorrect(0));
    assertTrue(verifyTemp.getIsPOVCountCorrect(0));
    assertTrue(verifyTemp.getIsJoystickNameCorrect(0));
  }
}
