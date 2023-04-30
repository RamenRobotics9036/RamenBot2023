package frc.robot.Simulation;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import frc.robot.Constants;

public class CalcArmAngleTest {
    private CalcArmAngle m_calcArmAngle;
    private final double toleranceDegrees = 2;
    private final double toleranceRotations = 0.01;

    @BeforeEach
    public void setUp() {
    }

    @Test
    public void ArmBeyondFullyUpShouldReturnError() {
      double stringLen = (CalcArmAngle.m_armHeightFromWinchToPivotPoint - CalcArmAngle.m_armLengthFromEdgeToPivot);
      double expectedResult = -1;

      assertTrue(CalcArmAngle.GetDegreesForStringLength(stringLen) == expectedResult);
    }

    @Test
    public void ArmBeyondFullyDownShouldReturnError() {
      double stringLen = (CalcArmAngle.m_armHeightFromWinchToPivotPoint + CalcArmAngle.m_armLengthFromEdgeToPivot);
      double expectedResult = -1;
    
      assertTrue(CalcArmAngle.GetDegreesForStringLength(stringLen) == expectedResult);
    }

    @Test
    public void ArmAtFullyUpShouldReturn90Degrees() {
      double stringLen = (CalcArmAngle.m_armHeightFromWinchToPivotPoint - CalcArmAngle.m_armLengthFromEdgeToPivot) + 0.00001;
      double expectedResult = 90;
    
      assertEquals(CalcArmAngle.GetDegreesForStringLength(stringLen), expectedResult, toleranceDegrees);
    }
    
    @Test
    public void ArmAtFullyDownShouldReturn270Degrees() {
      double stringLen = CalcArmAngle.m_armHeightFromWinchToPivotPoint + CalcArmAngle.m_armLengthFromEdgeToPivot - 0.00001;
      double expectedResult = 270;
    
      assertEquals(CalcArmAngle.GetDegreesForStringLength(stringLen), expectedResult, toleranceDegrees);
    }

    @Test
    public void LevelArmShouldReturn0Degrees() {
      double stringLen = CalcArmAngle.m_armHeightFromWinchToPivotPoint;
      double expectedResult = 0;
    
      assertEquals(CalcArmAngle.GetDegreesForStringLength(stringLen), expectedResult, toleranceDegrees);
    }

    @Test
    public void ArmAtFullyDownShouldReturnThreeQuartersRotations() {
      double stringLen = CalcArmAngle.m_armHeightFromWinchToPivotPoint + CalcArmAngle.m_armLengthFromEdgeToPivot - 0.00001;
      double expectedResult = 0.75;
    
      assertEquals(CalcArmAngle.GetRotationsForStringLength(stringLen), expectedResult, toleranceRotations);
    }
}

