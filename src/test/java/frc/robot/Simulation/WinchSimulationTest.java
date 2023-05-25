package frc.robot.Simulation;

import edu.wpi.first.hal.HAL;
import frc.robot.Subsystems.RelativeEncoderSim;
import frc.robot.Simulation.WinchSimulation.StringOrientation;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

public class WinchSimulationTest {
  private final double m_SpoolDiameterMeters = 0.01; // (1 centimeter)
  private final double m_TotalStringLenMeters = 5;
  private final double m_InitialLenSpooled = 1;
  private final StringOrientation m_InitialStringOrientation = StringOrientation.BackOfRobot;
  private final boolean m_invertMotor = false;

  private RelativeEncoderSim m_relEncoderSim;

  @BeforeEach
  public void setUp() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

    m_relEncoderSim = new RelativeEncoderSim(null, true); // test-mode
  }

  @Test
  public void NullDCMotorSimShouldThrowException() {
    assertThrows(IllegalArgumentException.class, () -> {
      WinchSimulation tempWinchSimulation = new WinchSimulation(null, m_SpoolDiameterMeters,
          m_TotalStringLenMeters, m_InitialLenSpooled, m_InitialStringOrientation, m_invertMotor);
    });
  }

  @Test
  public void InitialLenSpooledLessThanTotalStringLenShouldSucceed() {
    WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
        m_SpoolDiameterMeters, 5, 4, m_InitialStringOrientation, m_invertMotor);

    assertTrue(tempWinchSimulation != null);
  }

  @Test
  public void InitialLenSpooledLenEqualToTotalStringLenShouldSucceed() {
    WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
        m_SpoolDiameterMeters, 5, 5, m_InitialStringOrientation, m_invertMotor);

    assertTrue(tempWinchSimulation != null);
  }

  @Test
  public void InitialLenSpooledLenGreaterThanTotalStringLenShouldFail() {
    assertThrows(IllegalArgumentException.class, () -> {
      WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
          m_SpoolDiameterMeters, 5, 5.1, m_InitialStringOrientation, m_invertMotor);
    });
  }

  @Test
  public void InitialLenSpooledLenZeroShouldSucceed() {
    WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
        m_SpoolDiameterMeters, 5, 0, m_InitialStringOrientation, m_invertMotor);

    assertTrue(tempWinchSimulation != null);
  }

  @Test
  public void InitialLenSpooledLenLessThanZeroShouldFail() {
    assertThrows(IllegalArgumentException.class, () -> {
      WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
          m_SpoolDiameterMeters, 5, -1, m_InitialStringOrientation, m_invertMotor);
    });
  }

  @Test
  public void QueryingStringLenExtendedShouldReturnCorrectValueWhenStringOnBack() {
    double totalStringLen = 5;
    double stringLenSpooled = 1;
    StringOrientation stringOrientation = StringOrientation.BackOfRobot;

    WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
        m_SpoolDiameterMeters, totalStringLen, stringLenSpooled, stringOrientation, m_invertMotor);

    assertTrue(tempWinchSimulation.GetStringExtendedLen() == totalStringLen - stringLenSpooled);
  }

  @Test
  public void QueryingStringLenExtendedShouldReturnCorrectValueWhenStringOnFront() {
    double totalStringLen = 5;
    double stringLenSpooled = 1;
    StringOrientation stringOrientation = StringOrientation.FrontOfRobot;

    WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
        m_SpoolDiameterMeters, totalStringLen, stringLenSpooled, stringOrientation, m_invertMotor);

    assertTrue(tempWinchSimulation.GetStringExtendedLen() == totalStringLen - stringLenSpooled);
  }

  @Test
  public void QueryingStringOrientationFrontShouldReturnCorrectValue() {
    double initialLenSpooled = 1;
    StringOrientation stringOrientation = StringOrientation.FrontOfRobot;

    WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
        m_SpoolDiameterMeters, m_TotalStringLenMeters, initialLenSpooled, stringOrientation,
        m_invertMotor);

    assertTrue(tempWinchSimulation.GetStringOrientation() == stringOrientation);
  }

  @Test
  public void QueryingStringOrientationBackShouldReturnCorrectValue() {
    double initialLenSpooled = 1;
    StringOrientation stringOrientation = StringOrientation.BackOfRobot;

    WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
        m_SpoolDiameterMeters, m_TotalStringLenMeters, initialLenSpooled, stringOrientation,
        m_invertMotor);

    assertTrue(tempWinchSimulation.GetStringOrientation() == stringOrientation);
  }

  @Test
  public void QueryingStringOrientationShouldReturnBackWhenSpooledAmountIsZero() {
    StringOrientation stringOrientation = StringOrientation.FrontOfRobot;
    double initialLenSpooled = 0;

    WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
        m_SpoolDiameterMeters, m_TotalStringLenMeters, initialLenSpooled, stringOrientation,
        m_invertMotor);

    assertTrue(tempWinchSimulation.GetStringOrientation() == StringOrientation.BackOfRobot);
  }

  private void TestWinchMove(StringOrientation stringOrientation,
      double lenToGrowString,
      boolean flipWinchPolarity,
      double expectedResult,
      boolean expectIsBroken) {

    double tolerance = 0.01;

    WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
        m_SpoolDiameterMeters, m_TotalStringLenMeters, m_InitialLenSpooled, stringOrientation,
        flipWinchPolarity);

    // Rotate the motor such that string gets 0.5 meters longer
    double spoolCircumference = m_SpoolDiameterMeters * Math.PI;
    double numRotations = lenToGrowString / spoolCircumference;
    m_relEncoderSim.setPosition(numRotations);

    tempWinchSimulation.simulationPeriodic();
    double result = tempWinchSimulation.GetStringExtendedLen();

    assertEquals(result, expectedResult, tolerance);
    assertTrue(tempWinchSimulation.GetIsBroken() == expectIsBroken);
  }

  @Test
  public void WhenStringOnBackAndMotorTurnsClockwiseThenStringShouldGetLonger() {
    TestWinchMove(StringOrientation.BackOfRobot,
        0.5, // lenToGrowString
        false, // flipWinchPolarity
        4.5, // expectedResult
        false); // expectedIsBroken
  }

  @Test
  public void WhenStringOnBackAndMotorTurnsCounterClockwiseThenStringShouldGetShorter() {
    TestWinchMove(StringOrientation.BackOfRobot,
        -0.5, // lenToGrowString
        false, // flipWinchPolarity
        3.5, // expectedResult
        false); // expectedIsBroken
  }

  @Test
  public void WinchShouldMoveOtherDirectionWhenPolarityInverted() {
    TestWinchMove(StringOrientation.BackOfRobot,
        0.5, // lenToGrowString
        true, // flipWinchPolarity
        3.5, // expectedResult
        false); // expectedIsBroken
  }

  @Test
  public void WinchShouldSpoolOtherDirectionIfStringExtendedTooLong() {
    TestWinchMove(StringOrientation.BackOfRobot,
        1.5, // lenToGrowString
        false, // flipWinchPolarity
        4.5, // expectedResult
        false); // expectedIsBroken
  }

  @Test
  public void WinchShouldntBreakIfEntirelySpooled() {
    TestWinchMove(StringOrientation.BackOfRobot,
        5.99, // lenToGrowString
        false, // flipWinchPolarity
        0.01, // expectedResult
        false); // expectedIsBroken
  }

  @Test
  public void WinchShouldBreakIfOverSpooled() {
    TestWinchMove(StringOrientation.BackOfRobot,
        6.01, // lenToGrowString
        false, // flipWinchPolarity
        0.0, // expectedResult
        true); // expectedIsBroken
  }

  @Test
  public void TwoMotorMovesShouldMoveStringCumulatively() {
    double tolerance = 0.01;
    double expectedResult = 4.4;

    WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
        m_SpoolDiameterMeters, m_TotalStringLenMeters, m_InitialLenSpooled,
        StringOrientation.BackOfRobot, false);

    // Rotate the motor such that string gets a bit longer
    double spoolCircumference = m_SpoolDiameterMeters * Math.PI;
    double numRotations = 0.2 / spoolCircumference;
    m_relEncoderSim.setPosition(numRotations);
    tempWinchSimulation.simulationPeriodic();

    // Rotate again
    numRotations = 0.2 / spoolCircumference;
    m_relEncoderSim.setPosition(m_relEncoderSim.getPosition() + numRotations);
    tempWinchSimulation.simulationPeriodic();

    double result = tempWinchSimulation.GetStringExtendedLen();

    assertEquals(result, expectedResult, tolerance);
    assertTrue(!tempWinchSimulation.GetIsBroken());

  }
}
