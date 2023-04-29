package frc.robot.Simulation;

/* We make some simplifying assumptions for the simulation:
 *   1) Hardcoded length of arm, and distance from winch to point that string is attached to arm
 *   2) Assume that the point that string is attached to arm can slide up and down the arm, to ensure that the string is
 *      always perpendicular to the floor.  This isn't ideal, but simplifies the math as it ensures there's always
 *      a right triangle :)
 *
 * Reminder: 0 degrees points to the right, 90 straight up, 180 to left, 270 down
 */

public class CalcArmAngle {
  private final double m_armLengthMeters = 1.0; // For simulation, hardcoded length of arm
  private final double m_armLengthFromEdgeToPivot = 0.2;
  private final double m_armHeightFromWinchToPivotPoint = 1.0;

  // Constructor
  public CalcArmAngle() {
  }

  // We are calculating the angle of a right triangle at point (0,0).  We know the length of the hypotenus, and we know
  // it's other side is exactly H height above the x-axis
  //      /
  //    /  |
  //  /    |
  // -----------
  private double RightTriangleAngleGivenHeight(double lenHypotenuse, double height) {
    return Math.asin(height / lenHypotenuse);
  }

  // Returns a value between 0-360
  // Returns -1 if invalid string length
  public double GetDegreesForStringLength(double stringLen) {
    double backOfArmHeightAbovePivot = stringLen - m_armHeightFromWinchToPivotPoint;

    // Is arm beyond lowest possible point?
    if (backOfArmHeightAbovePivot >= m_armLengthFromEdgeToPivot) {
      return -1;
    }

    // Is arm beyond highest possible point?
    if (backOfArmHeightAbovePivot <= -1 * m_armLengthFromEdgeToPivot) {
      return -1;
    }

    // Level arm?
    if (backOfArmHeightAbovePivot == 0) {
      return 0;
    }
    else if (backOfArmHeightAbovePivot > 0) {
      return 360 - RightTriangleAngleGivenHeight(m_armLengthFromEdgeToPivot, backOfArmHeightAbovePivot);
    }
    else {
      return RightTriangleAngleGivenHeight(m_armLengthFromEdgeToPivot, Math.abs(backOfArmHeightAbovePivot));
    }
  }
}

