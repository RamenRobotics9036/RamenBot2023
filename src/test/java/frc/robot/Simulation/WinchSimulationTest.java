package frc.robot.Simulation;

import edu.wpi.first.hal.HAL;
import frc.robot.Subsystems.RelativeEncoderSim;
import frc.robot.Simulation.WinchSimulation.WindingOrientation;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

public class WinchSimulationTest {
  private final double m_SpoolDiameterMeters = 0.01; // (1 centimeter)
  private final double m_TotalStringLenMeters = 5;
  private final double m_InitialLenSpooled = 1;
  private final WindingOrientation m_InitialStringOrientation = WindingOrientation.BackOfRobot;
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
      assertTrue(tempWinchSimulation != null);
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
      assertTrue(tempWinchSimulation != null);
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
      assertTrue(tempWinchSimulation != null);
    });
  }

  @Test
  public void QueryingStringLenExtendedShouldReturnCorrectValueWhenStringOnBack() {
    double totalStringLen = 5;
    double stringLenSpooled = 1;
    WindingOrientation stringOrientation = WindingOrientation.BackOfRobot;

    WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
        m_SpoolDiameterMeters, totalStringLen, stringLenSpooled, stringOrientation, m_invertMotor);

    assertTrue(tempWinchSimulation.getStringUnspooledLen() == totalStringLen - stringLenSpooled);
  }

  @Test
  public void QueryingStringLenExtendedShouldReturnCorrectValueWhenStringOnFront() {
    double totalStringLen = 5;
    double stringLenSpooled = 1;
    WindingOrientation stringOrientation = WindingOrientation.FrontOfRobot;

    WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
        m_SpoolDiameterMeters, totalStringLen, stringLenSpooled, stringOrientation, m_invertMotor);

    assertTrue(tempWinchSimulation.getStringUnspooledLen() == totalStringLen - stringLenSpooled);
  }

  @Test
  public void QueryingStringOrientationFrontShouldReturnCorrectValue() {
    double initialLenSpooled = 1;
    WindingOrientation stringOrientation = WindingOrientation.FrontOfRobot;

    WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
        m_SpoolDiameterMeters, m_TotalStringLenMeters, initialLenSpooled, stringOrientation,
        m_invertMotor);

    assertTrue(tempWinchSimulation.getWindingOrientation() == stringOrientation);
  }

  @Test
  public void QueryingStringOrientationBackShouldReturnCorrectValue() {
    double initialLenSpooled = 1;
    WindingOrientation stringOrientation = WindingOrientation.BackOfRobot;

    WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
        m_SpoolDiameterMeters, m_TotalStringLenMeters, initialLenSpooled, stringOrientation,
        m_invertMotor);

    assertTrue(tempWinchSimulation.getWindingOrientation() == stringOrientation);
  }

  @Test
  public void QueryingStringOrientationShouldReturnBackWhenSpooledAmountIsZero() {
    WindingOrientation stringOrientation = WindingOrientation.FrontOfRobot;
    double initialLenSpooled = 0;

    WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
        m_SpoolDiameterMeters, m_TotalStringLenMeters, initialLenSpooled, stringOrientation,
        m_invertMotor);

    assertTrue(tempWinchSimulation.getWindingOrientation() == WindingOrientation.BackOfRobot);
  }

  private void TestWinchMove(WindingOrientation stringOrientation,
      double lenToGrowString,
      boolean flipWinchPolarity,
      double expectedResult,
      boolean expectIsBroken) {

    WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
        m_SpoolDiameterMeters, m_TotalStringLenMeters, m_InitialLenSpooled, stringOrientation,
        flipWinchPolarity);

    // Rotate the motor such that string gets 0.5 meters longer
    double spoolCircumference = m_SpoolDiameterMeters * Math.PI;
    double numRotations = lenToGrowString / spoolCircumference;
    m_relEncoderSim.setPosition(numRotations);

    tempWinchSimulation.simulationPeriodic();
    double result = tempWinchSimulation.getStringUnspooledLen();

    assertEquals(result, expectedResult, UnitConversions.kAngleTolerance);
    assertTrue(tempWinchSimulation.getIsBroken() == expectIsBroken);
  }

  @Test
  public void WhenStringOnBackAndMotorTurnsClockwiseThenStringShouldGetLonger() {
    TestWinchMove(WindingOrientation.BackOfRobot,
        0.5, // lenToGrowString
        false, // flipWinchPolarity
        4.5, // expectedResult
        false); // expectedIsBroken
  }

  @Test
  public void WhenStringOnBackAndMotorTurnsCounterClockwiseThenStringShouldGetShorter() {
    TestWinchMove(WindingOrientation.BackOfRobot,
        -0.5, // lenToGrowString
        false, // flipWinchPolarity
        3.5, // expectedResult
        false); // expectedIsBroken
  }

  @Test
  public void WinchShouldMoveOtherDirectionWhenPolarityInverted() {
    TestWinchMove(WindingOrientation.BackOfRobot,
        0.5, // lenToGrowString
        true, // flipWinchPolarity
        3.5, // expectedResult
        false); // expectedIsBroken
  }

  @Test
  public void WinchShouldSpoolOtherDirectionIfStringExtendedTooLong() {
    TestWinchMove(WindingOrientation.BackOfRobot,
        1.5, // lenToGrowString
        false, // flipWinchPolarity
        4.5, // expectedResult
        false); // expectedIsBroken
  }

  @Test
  public void WinchShouldntBreakIfEntirelySpooled() {
    TestWinchMove(WindingOrientation.BackOfRobot,
        5.99, // lenToGrowString
        false, // flipWinchPolarity
        0.01, // expectedResult
        false); // expectedIsBroken
  }

  @Test
  public void WinchShouldBreakIfOverSpooled() {
    TestWinchMove(WindingOrientation.BackOfRobot,
        6.01, // lenToGrowString
        false, // flipWinchPolarity
        0.0, // expectedResult
        true); // expectedIsBroken
  }

  @Test
  public void TwoMotorMovesShouldMoveStringCumulatively() {
    double expectedResult = 4.4;

    WinchSimulation tempWinchSimulation = new WinchSimulation(m_relEncoderSim,
        m_SpoolDiameterMeters, m_TotalStringLenMeters, m_InitialLenSpooled,
        WindingOrientation.BackOfRobot, false);

    // Rotate the motor such that string gets a bit longer
    double spoolCircumference = m_SpoolDiameterMeters * Math.PI;
    double numRotations = 0.2 / spoolCircumference;
    m_relEncoderSim.setPosition(numRotations);
    tempWinchSimulation.simulationPeriodic();

    // Rotate again
    numRotations = 0.2 / spoolCircumference;
    m_relEncoderSim.setPosition(m_relEncoderSim.getPosition() + numRotations);
    tempWinchSimulation.simulationPeriodic();

    double result = tempWinchSimulation.getStringUnspooledLen();

    assertEquals(result, expectedResult, UnitConversions.kAngleTolerance);
    assertTrue(!tempWinchSimulation.getIsBroken());

  }
}
