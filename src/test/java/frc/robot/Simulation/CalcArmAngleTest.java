package frc.robot.Simulation;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import frc.robot.Constants;

public class CalcArmAngleTest {
    private final double toleranceDegrees = 2;
    private final double toleranceRotations = 0.01;
    private CalcArmAngle m_calcArmAngle;
    private final double m_armHeightFromWinchToPivotPoint = 1;
    private final double m_armLengthFromEdgeToPivot = 0.25;

    @BeforeEach
    public void setUp() {
      m_calcArmAngle = new CalcArmAngle(m_armHeightFromWinchToPivotPoint, m_armLengthFromEdgeToPivot);
    }

    @Test
    public void ArmBeyondFullyUpShouldReturnError() {
      double amountBeyondLimit = 0.0001;

      double stringLen = (m_armHeightFromWinchToPivotPoint - m_armLengthFromEdgeToPivot - amountBeyondLimit);
      double expectedResult = -1;

      assertTrue(m_calcArmAngle.GetDegreesForStringLength(stringLen) == expectedResult);
    }

    @Test
    public void ArmBeyondFullyDownShouldReturnError() {
      double amountBeyondLimit = 0.0001;

      double stringLen = (m_armHeightFromWinchToPivotPoint + m_armLengthFromEdgeToPivot + amountBeyondLimit);
      double expectedResult = -1;
    
      assertTrue(m_calcArmAngle.GetDegreesForStringLength(stringLen) == expectedResult);
    }

    @Test
    public void ArmAtFullyUpShouldReturn90Degrees() {
      double stringLen = (m_armHeightFromWinchToPivotPoint - m_armLengthFromEdgeToPivot);
      double expectedResult = 90;
    
      assertEquals(m_calcArmAngle.GetDegreesForStringLength(stringLen), expectedResult, toleranceDegrees);
    }
    
    @Test
    public void ArmAtFullyDownShouldReturn270Degrees() {
      double stringLen = m_armHeightFromWinchToPivotPoint + m_armLengthFromEdgeToPivot;
      double expectedResult = 270;
    
      assertEquals(m_calcArmAngle.GetDegreesForStringLength(stringLen), expectedResult, toleranceDegrees);
    }

    @Test
    public void LevelArmShouldReturn0Degrees() {
      double stringLen = m_armHeightFromWinchToPivotPoint;
      double expectedResult = 0;
    
      assertEquals(m_calcArmAngle.GetDegreesForStringLength(stringLen), expectedResult, toleranceDegrees);
    }

    @Test
    public void ArmAtFullyDownShouldReturnThreeQuartersRotations() {
      double stringLen = m_armHeightFromWinchToPivotPoint + m_armLengthFromEdgeToPivot;
      double expectedResult = 0.75;
    
      assertEquals(m_calcArmAngle.GetRotationsForStringLength(stringLen), expectedResult, toleranceRotations);
    }

    @Test
    public void ArmAt45DegreesShouldSucceed() {
      double stringLen = (m_armHeightFromWinchToPivotPoint - 0.17678);
      double expectedResult = 45;

      assertEquals(m_calcArmAngle.GetDegreesForStringLength(stringLen), expectedResult, toleranceDegrees);
    }
}

