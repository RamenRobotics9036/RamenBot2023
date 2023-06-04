package frc.robot.simulation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import frc.robot.simulation.WinchSimulation.WindingOrientation;
import frc.robot.subsystems.RelativeEncoderSim;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests the WinchSimulation class.
 */
public class WinchSimulationTest {
  private final double m_spoolDiameterMeters = 0.01; // (1 centimeter)
  private final double m_totalStringLenMeters = 5;
  private final double m_initialLenSpooled = 1;
  private final WindingOrientation m_initialStringOrientation = WindingOrientation.BackOfRobot;
  private final boolean m_invertMotor = false;

  private RelativeEncoderSim m_relEncoderSim;

  /**
   * Runs before each test.
   */
  @BeforeEach
  public void setUp() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

    m_relEncoderSim = new RelativeEncoderSim(null, true); // test-mode
  }

  @Test
  public void nullDcMotorSimShouldThrowException() {
    assertThrows(IllegalArgumentException.class, () -> {
      WinchSimulation tempWinchSimulation = new WinchSimulation(null, m_spoolDiameterMeters,
          m_totalStringLenMeters, m_initialLenSpooled, m_initialStringOrientation, m_invertMotor);
      assertTrue(tempWinchSimulation != null);
    });
  }

  @Test
  public void initialLenSpooledLessThanTotalStringLenShouldSucceed() {
    WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
        m_spoolDiameterMeters, 5, 4, m_initialStringOrientation, m_invertMotor);

    assertTrue(tempWinchSimulation != null);
  }

  @Test
  public void initialLenSpooledLenEqualToTotalStringLenShouldSucceed() {
    WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
        m_spoolDiameterMeters, 5, 5, m_initialStringOrientation, m_invertMotor);

    assertTrue(tempWinchSimulation != null);
  }

  @Test
  public void initialLenSpooledLenGreaterThanTotalStringLenShouldFail() {
    assertThrows(IllegalArgumentException.class, () -> {
      WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
          m_spoolDiameterMeters, 5, 5.1, m_initialStringOrientation, m_invertMotor);
      assertTrue(tempWinchSimulation != null);
    });
  }

  @Test
  public void initialLenSpooledLenZeroShouldSucceed() {
    WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
        m_spoolDiameterMeters, 5, 0, m_initialStringOrientation, m_invertMotor);

    assertTrue(tempWinchSimulation != null);
  }

  @Test
  public void initialLenSpooledLenLessThanZeroShouldFail() {
    assertThrows(IllegalArgumentException.class, () -> {
      WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
          m_spoolDiameterMeters, 5, -1, m_initialStringOrientation, m_invertMotor);
      assertTrue(tempWinchSimulation != null);
    });
  }

  @Test
  public void queryingStringLenExtendedShouldReturnCorrectValueWhenStringOnBack() {
    double totalStringLen = 5;
    double stringLenSpooled = 1;
    WindingOrientation stringOrientation = WindingOrientation.BackOfRobot;

    WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
        m_spoolDiameterMeters, totalStringLen, stringLenSpooled, stringOrientation, m_invertMotor);

    assertTrue(tempWinchSimulation.getStringUnspooledLen() == totalStringLen - stringLenSpooled);
  }

  @Test
  public void queryingStringLenExtendedShouldReturnCorrectValueWhenStringOnFront() {
    double totalStringLen = 5;
    double stringLenSpooled = 1;
    WindingOrientation stringOrientation = WindingOrientation.FrontOfRobot;

    WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
        m_spoolDiameterMeters, totalStringLen, stringLenSpooled, stringOrientation, m_invertMotor);

    assertTrue(tempWinchSimulation.getStringUnspooledLen() == totalStringLen - stringLenSpooled);
  }

  @Test
  public void queryingStringOrientationFrontShouldReturnCorrectValue() {
    double initialLenSpooled = 1;
    WindingOrientation stringOrientation = WindingOrientation.FrontOfRobot;

    WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
        m_spoolDiameterMeters, m_totalStringLenMeters, initialLenSpooled, stringOrientation,
        m_invertMotor);

    assertTrue(tempWinchSimulation.getWindingOrientation() == stringOrientation);
  }

  @Test
  public void queryingStringOrientationBackShouldReturnCorrectValue() {
    double initialLenSpooled = 1;
    WindingOrientation stringOrientation = WindingOrientation.BackOfRobot;

    WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
        m_spoolDiameterMeters, m_totalStringLenMeters, initialLenSpooled, stringOrientation,
        m_invertMotor);

    assertTrue(tempWinchSimulation.getWindingOrientation() == stringOrientation);
  }

  @Test
  public void queryingStringOrientationShouldReturnBackWhenSpooledAmountIsZero() {
    WindingOrientation stringOrientation = WindingOrientation.FrontOfRobot;
    double initialLenSpooled = 0;

    WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
        m_spoolDiameterMeters, m_totalStringLenMeters, initialLenSpooled, stringOrientation,
        m_invertMotor);

    assertTrue(tempWinchSimulation.getWindingOrientation() == WindingOrientation.BackOfRobot);
  }

  private void testWinchMove(WindingOrientation stringOrientation,
      double lenToGrowString,
      boolean flipWinchPolarity,
      double expectedResult,
      boolean expectIsBroken) {

    WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
        m_spoolDiameterMeters, m_totalStringLenMeters, m_initialLenSpooled, stringOrientation,
        flipWinchPolarity);

    // Rotate the motor such that string gets 0.5 meters longer
    double spoolCircumference = m_spoolDiameterMeters * Math.PI;
    double numRotations = lenToGrowString / spoolCircumference;
    m_relEncoderSim.setPosition(numRotations);

    tempWinchSimulation.simulationPeriodic();
    double result = tempWinchSimulation.getStringUnspooledLen();

    assertEquals(result, expectedResult, UnitConversions.kAngleTolerance);
    assertTrue(tempWinchSimulation.getIsBroken() == expectIsBroken);
  }

  @Test
  public void whenStringOnBackAndMotorTurnsClockwiseThenStringShouldGetLonger() {
    testWinchMove(WindingOrientation.BackOfRobot,
        0.5, // lenToGrowString
        false, // flipWinchPolarity
        4.5, // expectedResult
        false); // expectedIsBroken
  }

  @Test
  public void whenStringOnBackAndMotorTurnsCounterClockwiseThenStringShouldGetShorter() {
    testWinchMove(WindingOrientation.BackOfRobot,
        -0.5, // lenToGrowString
        false, // flipWinchPolarity
        3.5, // expectedResult
        false); // expectedIsBroken
  }

  @Test
  public void winchShouldMoveOtherDirectionWhenPolarityInverted() {
    testWinchMove(WindingOrientation.BackOfRobot,
        0.5, // lenToGrowString
        true, // flipWinchPolarity
        3.5, // expectedResult
        false); // expectedIsBroken
  }

  @Test
  public void winchShouldSpoolOtherDirectionIfStringExtendedTooLong() {
    testWinchMove(WindingOrientation.BackOfRobot,
        1.5, // lenToGrowString
        false, // flipWinchPolarity
        4.5, // expectedResult
        false); // expectedIsBroken
  }

  @Test
  public void winchShouldntBreakIfEntirelySpooled() {
    testWinchMove(WindingOrientation.BackOfRobot,
        5.99, // lenToGrowString
        false, // flipWinchPolarity
        0.01, // expectedResult
        false); // expectedIsBroken
  }

  @Test
  public void winchShouldBreakIfOverSpooled() {
    testWinchMove(WindingOrientation.BackOfRobot,
        6.01, // lenToGrowString
        false, // flipWinchPolarity
        0.0, // expectedResult
        true); // expectedIsBroken
  }

  @Test
  public void twoMotorMovesShouldMoveStringCumulatively() {
    WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
        m_spoolDiameterMeters, m_totalStringLenMeters, m_initialLenSpooled,
        WindingOrientation.BackOfRobot, false);

    // Rotate the motor such that string gets a bit longer
    double spoolCircumference = m_spoolDiameterMeters * Math.PI;
    double numRotations = 0.2 / spoolCircumference;
    m_relEncoderSim.setPosition(numRotations);
    tempWinchSimulation.simulationPeriodic();

    // Rotate again
    numRotations = 0.2 / spoolCircumference;
    m_relEncoderSim.setPosition(m_relEncoderSim.getPosition() + numRotations);
    tempWinchSimulation.simulationPeriodic();

    double result = tempWinchSimulation.getStringUnspooledLen();

    double expectedResult = 4.4;
    assertEquals(result, expectedResult, UnitConversions.kAngleTolerance);
    assertTrue(!tempWinchSimulation.getIsBroken());

  }
}
