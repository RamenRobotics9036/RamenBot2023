package frc.robot.simulation;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertEquals;

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
  public void ArmBeyondFullyUpShouldReturnError() {
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
  public void ArmBeyondFullyDownShouldReturnSuccessSinceStringDangling() {
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
  public void ArmAtFullyUpShouldReturn90Degrees() {
    double stringLen = (m_armHeightFromWinchToPivotPoint - m_armLengthFromEdgeToPivot);
    double expectedResult = 90;

    assertEquals(m_calcArmAngleHelper.calcSignedDegreesForStringLength(stringLen).m_value,
        expectedResult,
        UnitConversions.kAngleTolerance);
  }

  @Test
  public void ArmAtFullyDownShouldReturnNegative90Degrees() {
    double stringLen = m_armHeightFromWinchToPivotPoint + m_armLengthFromEdgeToPivot;
    double expectedResult = -90;

    assertEquals(m_calcArmAngleHelper.calcSignedDegreesForStringLength(stringLen).m_value,
        expectedResult,
        UnitConversions.kAngleTolerance);
  }

  @Test
  public void LevelArmShouldReturn0Degrees() {
    double stringLen = m_armHeightFromWinchToPivotPoint;
    double expectedResult = 0;

    assertEquals(m_calcArmAngleHelper.calcSignedDegreesForStringLength(stringLen).m_value,
        expectedResult,
        UnitConversions.kAngleTolerance);
  }

  @Test
  public void ArmAt45DegreesShouldSucceed() {
    double stringLen = (m_armHeightFromWinchToPivotPoint - 0.17678);
    double expectedResult = 45;

    assertEquals(m_calcArmAngleHelper.calcSignedDegreesForStringLength(stringLen).m_value,
        expectedResult,
        UnitConversions.kAngleTolerance);
  }
}