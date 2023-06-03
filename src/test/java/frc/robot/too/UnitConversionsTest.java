package frc.robot.Simulation;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;

public class UnitConversionsTest {
  @BeforeEach
  public void setUp() {
  }

  @Test
  public void toSignedDegreesTestZeroDegrees() {
    double input = 0;
    double expected = 0;
    assertEquals(expected, UnitConversions.toSignedDegrees(input), 0.001);
  }

  @Test
  public void toSignedDegreesTestPositiveDegrees() {
    double input = 45;
    double expected = 45;
    assertEquals(expected, UnitConversions.toSignedDegrees(input), 0.001);
  }

  @Test
  public void toSignedDegreesTestNegativeDegrees() {
    double input = -45;
    double expected = -45;
    assertEquals(expected, UnitConversions.toSignedDegrees(input), 0.001);
  }

  @Test
  public void toSignedDegreesTestBelowAxis() {
    double input = 350;
    double expected = -10;
    assertEquals(expected, UnitConversions.toSignedDegrees(input), 0.001);
  }

  @Test
  public void toSignedDegreesTestWrapAroundPositiveDegrees() {
    double input = 370;
    double expected = 10;
    assertEquals(expected, UnitConversions.toSignedDegrees(input), 0.001);
  }

  @Test
  public void toSignedDegreesTestWrapAroundNegativeDegrees() {
    double input = -370;
    double expected = -10;
    assertEquals(expected, UnitConversions.toSignedDegrees(input), 0.001);
  }

  @Test
  void toUnSignedDegreesTestToUnsignedDegrees_Positive() {
    double signedDegrees = 45;
    double expectedUnsignedDegrees = 45;
    assertEquals(expectedUnsignedDegrees, UnitConversions.toUnsignedDegrees(signedDegrees), 0.001);
  }

  @Test
  void toUnSignedDegreesTestToUnsignedDegrees_Negative() {
    double signedDegrees = -45;
    double expectedUnsignedDegrees = 315;
    assertEquals(expectedUnsignedDegrees, UnitConversions.toUnsignedDegrees(signedDegrees), 0.001);
  }

  @Test
  void toUnSignedDegreesTestToUnsignedDegrees_GreaterThan360() {
    double signedDegrees = 361;
    double expectedUnsignedDegrees = 1;
    assertEquals(expectedUnsignedDegrees, UnitConversions.toUnsignedDegrees(signedDegrees), 0.001);
  }

  @Test
  void toUnSignedDegreesTestToUnsignedDegrees_LessThanNegative360() {
    double signedDegrees = -361;
    double expectedUnsignedDegrees = 359;
    assertEquals(expectedUnsignedDegrees, UnitConversions.toUnsignedDegrees(signedDegrees), 0.001);
  }

  @Test
  void toUnSignedDegreesTestToUnsignedDegrees_MultipleOf360() {
    double signedDegrees = 720;
    double expectedUnsignedDegrees = 0;
    assertEquals(expectedUnsignedDegrees, UnitConversions.toUnsignedDegrees(signedDegrees), 0.001);
  }
}
