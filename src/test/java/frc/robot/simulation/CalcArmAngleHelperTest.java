package frc.robot.simulation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.robot.simulation.CalcArmAngleHelper.Result;

/**
 * Tests the CalcArmAngleHelper class.
 */
public class CalcArmAngleHelperTest {
  private CalcArmAngleHelper m_calcArmAngleHelper;
  private final double m_armHeightFromWinchToPivotPoint = 1;
  private final double m_armLengthFromEdgeToPivot = 0.25;

  @BeforeEach
  public void setUp() {
    m_calcArmAngleHelper = new CalcArmAngleHelper(m_armHeightFromWinchToPivotPoint,
        m_armLengthFromEdgeToPivot);
  }

  @Test
  public void calcDegreesArmBeyondFullyUpShouldReturnError() {
    double amountBeyondLimit = 0.0001;

    double stringLen = (m_armHeightFromWinchToPivotPoint - m_armLengthFromEdgeToPivot
        - amountBeyondLimit);
    double expectedResult = 90;

    CalcArmAngleHelper.Result resultPair = m_calcArmAngleHelper
        .calcSignedDegreesForStringLength(stringLen);
    assertTrue(!resultPair.m_isValid);
    assertTrue(resultPair.m_value == expectedResult);
  }

  @Test
  public void calcDegreesArmBeyondFullyDownShouldReturnSuccessSinceStringDangling() {
    double amountBeyondLimit = 0.0001;

    double stringLen = (m_armHeightFromWinchToPivotPoint + m_armLengthFromEdgeToPivot
        + amountBeyondLimit);
    double expectedResult = -90;

    CalcArmAngleHelper.Result resultPair = m_calcArmAngleHelper
        .calcSignedDegreesForStringLength(stringLen);
    assertTrue(resultPair.m_isValid);
    assertTrue(resultPair.m_value == expectedResult);
  }

  @Test
  public void calcDegreesArmAtFullyUpShouldReturn90Degrees() {
    double stringLen = (m_armHeightFromWinchToPivotPoint - m_armLengthFromEdgeToPivot);
    double expectedResult = 90;

    assertEquals(m_calcArmAngleHelper.calcSignedDegreesForStringLength(stringLen).m_value,
        expectedResult,
        UnitConversions.kAngleTolerance);
  }

  @Test
  public void calcDegreesArmAtFullyDownShouldReturnNegative90Degrees() {
    double stringLen = m_armHeightFromWinchToPivotPoint + m_armLengthFromEdgeToPivot;
    double expectedResult = -90;

    assertEquals(m_calcArmAngleHelper.calcSignedDegreesForStringLength(stringLen).m_value,
        expectedResult,
        UnitConversions.kAngleTolerance);
  }

  @Test
  public void calcDegreesLevelArmShouldReturn0Degrees() {
    double stringLen = m_armHeightFromWinchToPivotPoint;
    double expectedResult = 0;

    assertEquals(m_calcArmAngleHelper.calcSignedDegreesForStringLength(stringLen).m_value,
        expectedResult,
        UnitConversions.kAngleTolerance);
  }

  @Test
  public void calcDegreesArmAt45DegreesShouldSucceed() {
    double stringLen = (m_armHeightFromWinchToPivotPoint - 0.17678);
    double expectedResult = 45;

    assertEquals(m_calcArmAngleHelper.calcSignedDegreesForStringLength(stringLen).m_value,
        expectedResult,
        UnitConversions.kAngleTolerance);
  }

  @Test
  public void calcStringLenFor0Degrees() {
    double degrees = 0;
    double expectedResult = m_armHeightFromWinchToPivotPoint;

    Result result = m_calcArmAngleHelper.calcStringLengthForSignedDegrees(degrees);

    assertEquals(result.m_value, expectedResult, UnitConversions.kDoubleTolerance);
    assertTrue(result.m_isValid);
  }

  @Test
  public void calcStringLenFor90Degrees() {
    double degrees = 90;
    double expectedResult = m_armHeightFromWinchToPivotPoint - m_armLengthFromEdgeToPivot;

    Result result = m_calcArmAngleHelper.calcStringLengthForSignedDegrees(degrees);

    assertEquals(result.m_value, expectedResult, UnitConversions.kDoubleTolerance);
    assertTrue(result.m_isValid);
  }

  @Test
  public void calcStringLenForNegative90Degrees() {
    double degrees = -90;
    double expectedResult = m_armHeightFromWinchToPivotPoint + m_armLengthFromEdgeToPivot;

    Result result = m_calcArmAngleHelper.calcStringLengthForSignedDegrees(degrees);

    assertEquals(result.m_value, expectedResult, UnitConversions.kDoubleTolerance);
    assertTrue(result.m_isValid);
  }
}
