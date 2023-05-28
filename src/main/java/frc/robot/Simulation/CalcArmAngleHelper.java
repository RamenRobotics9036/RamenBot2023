package frc.robot.Simulation;

import frc.robot.Constants;

/*
 * Parts of the robot:
 *
 * Winch - The winch is the motor that pulls the string
 * Arm - Robot arm, which rotates around the pivot point
 * Pivot Point - The pivot point is the point where the arm rotates around
 * Arm Back End - The back end of the arm, which is the end that is furthest away from the grabber
 * Arm Angle - The angle of the arm, relative to the ground
 * String - Connects winch to arm back end
 */

public class CalcArmAngleHelper {
  public class Result {
    public final boolean m_isValid;
    public final double m_value;

    public Result(boolean isValid, double value) {
      this.m_isValid = isValid;
      this.m_value = value;
    }
  }

  private double m_lengthFromWinchToPivotPoint;
  private double m_lengthFromPivotPointToArmBackEnd;

  // Constructor
  public CalcArmAngleHelper(double lengthFromWinchToPivotPoint,
      double lengthFromPivotPointToArmBackEnd) {
    if (lengthFromWinchToPivotPoint < Constants.SimConstants.klengthFromWinchToPivotPoint_Min) {
      throw new IllegalArgumentException(
          "Distance from winch to arm pivot point needs to be at least "
              + Constants.SimConstants.klengthFromWinchToPivotPoint_Min + " meters");
    }

    if (lengthFromPivotPointToArmBackEnd < Constants.SimConstants.klengthFromPivotPointToArmBackEnd_Min) {
      throw new IllegalArgumentException(
          "Length from arm pivot point to arm back end needs to be at least "
              + Constants.SimConstants.klengthFromPivotPointToArmBackEnd_Min
              + " meters, otherwise the arm cant be rotated");
    }

    m_lengthFromWinchToPivotPoint = lengthFromWinchToPivotPoint;
    m_lengthFromPivotPointToArmBackEnd = lengthFromPivotPointToArmBackEnd;
  }

  public Result CalcSignedDegreesForStringLength(double stringLen) {
    double heightArmBackendAbovePivot = stringLen - m_lengthFromWinchToPivotPoint;
    double up = 90;
    double down = -90;

    // Is arm beyond lowest possible point?
    // If the string is too long, it means the string is no longer taut.
    // Still, we consider this a valid position of the arm; arm is dangling down
    if (UnitConversions.greaterThanButNotEqual(heightArmBackendAbovePivot,
        m_lengthFromPivotPointToArmBackEnd)) {
      System.out.println("String too long, and is no longer taut");
      return new Result(true, down);
    }

    // Is arm beyond highest possible point?
    if (UnitConversions.lessThanButNotEqual(heightArmBackendAbovePivot,
        -1 * m_lengthFromPivotPointToArmBackEnd)) {
      System.out.println("Above highest point: String too short!");
      return new Result(false, up);
    }

    return new Result(true, -1
        * CalcAngleOnRightTriangle(m_lengthFromPivotPointToArmBackEnd, heightArmBackendAbovePivot));
  }

  // We are calculating the angle of a right triangle at point (0,0). We know the length of the
  // hypotenus, and we know it's other side is exactly H height above the x-axis
  private double CalcAngleOnRightTriangle(double lenHypotenuse, double height) {
    return Math.toDegrees(Math.asin(height / lenHypotenuse));
  }
}
