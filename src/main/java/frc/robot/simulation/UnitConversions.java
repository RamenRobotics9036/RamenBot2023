package frc.robot.Simulation;

/**
 * This class provides utility methods for performing various unit conversions.
 */
public class UnitConversions {
  public static final double kDoubleTolerance = 1E-10;
  public static final double kAngleTolerance = 1E-2;

  /**
   * Checks if two double values are equal within a specified tolerance.
   *
   * @param a The first double value to compare.
   * @param b The second double value to compare.
   * @return True if the absolute difference between the two values is less than the specified
   *         tolerance.
   */
  public static boolean areEqual(double a, double b) {
    return Math.abs(a - b) < kDoubleTolerance;
  }

  /**
   * Checks if a double value is less than or equal to another.
   *
   * @param a The first double value to compare.
   * @param b The second double value to compare.
   * @return True if a is less than or equal to b.
   */
  public static boolean lessThanOrEqualDouble(double a, double b) {
    if (areEqual(a, b)) {
      return true;
    }

    return a <= b;
  }

  /**
   * Checks if a double value is greater than or equal to another.
   *
   * @param a The first double value to compare.
   * @param b The second double value to compare.
   * @return True if a is greater than or equal to b.
   */
  public static boolean greaterThanOrEqualDouble(double a, double b) {
    if (areEqual(a, b)) {
      return true;
    }

    return a >= b;
  }

  /**
   * Checks if a double value is less than but not equal to another.
   *
   * @param a The first double value to compare.
   * @param b The second double value to compare.
   * @return True if a is less than and not equal to b.
   */
  public static boolean lessThanButNotEqualDouble(double a, double b) {
    if (areEqual(a, b)) {
      return false;
    }

    return a < b;
  }

  /**
   * Checks if a double value is greater than but not equal to another.
   *
   * @param a The first double value to compare.
   * @param b The second double value to compare.
   * @return True if a is greater than and not equal to b.
   */
  public static boolean greaterThanButNotEqualDouble(double a, double b) {
    if (areEqual(a, b)) {
      return false;
    }

    return a > b;
  }

  /**
   * Converts a given angle in degrees to signed degrees in the range (-180, 180].
   *
   * @param degrees The angle in degrees to be converted.
   * @return The converted angle in signed degrees.
   */
  public static double toSignedDegrees(double degrees) {
    double signedDegrees = degrees % 360;

    if (signedDegrees > 180) {
      signedDegrees -= 360;
    }
    else if (signedDegrees < -180) {
      signedDegrees += 360;
    }
    return signedDegrees;
  }

  /**
   * Converts a given angle in signed degrees to unsigned degrees in the range [0, 360).
   *
   * @param signedDegrees The angle in signed degrees to be converted.
   */
  public static double toUnsignedDegrees(double signedDegrees) {
    double unsignedDegrees = signedDegrees % 360;

    if (unsignedDegrees < 0) {
      unsignedDegrees += 360;
    }
    return unsignedDegrees;
  }

  /**
   * Returns true if the signed angle is between [-180, 180], known as the
   * right half plane.
   *
   * @param signedDegrees The angle in signed degrees.
   */
  public static boolean isInRightHalfPlane(double signedDegrees) {
    return signedDegrees >= -90 && signedDegrees <= 90;
  }
}
