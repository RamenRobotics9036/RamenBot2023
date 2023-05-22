package frc.robot.Simulation;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/* We make some simplifying assumptions for the simulation:
 *   1) Hardcoded length of arm, and distance from winch to point that string is attached to arm
 *   2) Assume that the point that string is attached to arm can slide up and down the arm, to ensure that the string is
 *      always perpendicular to the floor.  This isn't ideal, but simplifies the math as it ensures there's always
 *      a right triangle :)
 *
 * Reminder: 0 degrees points to the right, 90 straight up, 180 to left, 270 down
 */

public class CalcArmAngle {
  public class Result {
    public final boolean m_isValid;
    public final double m_value;

    public Result(boolean isValid, double value) {
      this.m_isValid = isValid;
      this.m_value = value;
    }
  }

  private double m_heightFromWinchToPivotPoint;
  private double m_armLengthFromEdgeToPivot;

  // Constructor
  public CalcArmAngle(double heightFromWinchToPivotPoint, double armLengthFromEdgeToPivot ) {
    if (heightFromWinchToPivotPoint < Constants.SimConstants.karmHeightFromWinchToPivotPoint_Min) {
      throw new IllegalArgumentException("Height from winch to arm needs to be at least "
        + Constants.SimConstants.karmHeightFromWinchToPivotPoint_Min
        + " meters");
    }

    if (armLengthFromEdgeToPivot < Constants.SimConstants.karmLengthFromEdgeToPivot_Min) {
      throw new IllegalArgumentException("armLengthFromEdgeToPivot needs to be at least "
        + Constants.SimConstants.karmLengthFromEdgeToPivot_Min
        + " meters, otherwise the arm cant be rotated");
    }

    m_heightFromWinchToPivotPoint = heightFromWinchToPivotPoint;
    m_armLengthFromEdgeToPivot = armLengthFromEdgeToPivot;
  }

  // We are calculating the angle of a right triangle at point (0,0).  We know the length of the hypotenus, and we know
  // it's other side is exactly H height above the x-axis
  //      /
  //    /  |
  //  /    |
  // -----------
  private double RightTriangleAngleGivenHeight(double lenHypotenuse, double height) {
    return Math.toDegrees(Math.asin(height / lenHypotenuse));
  }

  // Returns a value between 0-360
  public Result GetDegreesForStringLength(double stringLen) {
    double backOfArmHeightAbovePivot = stringLen - m_heightFromWinchToPivotPoint;

    // Is arm beyond lowest possible point?
    // If the string is too long, that's ok: it doesnt mean the arm is broken
    if (backOfArmHeightAbovePivot > m_armLengthFromEdgeToPivot) {
      System.out.println("Below lowest point: String too long!");      
      return new Result(true, 360 - 90);
    }

    // Is arm beyond highest possible point?
    if (backOfArmHeightAbovePivot < -1 * m_armLengthFromEdgeToPivot) {
      System.out.println("Above highest point: String too short!");
      return new Result(false, 90);
    }

    // Level arm?
    if (backOfArmHeightAbovePivot == 0) {
      return new Result(true, 0);
    }
    else if (backOfArmHeightAbovePivot > 0) {
      return new Result(true, 360 - RightTriangleAngleGivenHeight(m_armLengthFromEdgeToPivot, backOfArmHeightAbovePivot));
    }
    else {
      return new Result(true, RightTriangleAngleGivenHeight(m_armLengthFromEdgeToPivot, Math.abs(backOfArmHeightAbovePivot)));
    }
  }

  public Result GetRotationsForStringLength(double stringLen) {
    Result resultPair;

    resultPair =  GetDegreesForStringLength(stringLen);
    if (!resultPair.m_isValid) {
      return new Result(false, Units.degreesToRotations(resultPair.m_value));
    }
    else {  
      return new Result(true, Units.degreesToRotations(resultPair.m_value));
    }
  }
}
