package frc.robot.Simulation;

public class UnitConversions {
  public static final double kDoubleTolerance = 1E-10;
  public static final double kAngleTolerance = 1E-2;

  public static boolean areEqual(double a, double b) {
    return Math.abs(a - b) < kDoubleTolerance;
  }

  public static boolean lessThanOrEqual(double a, double b) {
    if (areEqual(a, b)) {
      return true;
    }

    return a <= b;
  }

  public static boolean greaterThanOrEqual(double a, double b) {
    if (areEqual(a, b)) {
      return true;
    }

    return a >= b;
  }

  public static boolean lessThanButNotEqual(double a, double b) {
    if (areEqual(a, b)) {
      return false;
    }

    return a < b;
  }

  public static boolean greaterThanButNotEqual(double a, double b) {
    if (areEqual(a, b)) {
      return false;
    }

    return a > b;
  }

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

  public static double toUnsignedDegrees(double signedDegrees) {
    double unsignedDegrees = signedDegrees % 360;

    if (unsignedDegrees < 0) {
      unsignedDegrees += 360;
    }
    return unsignedDegrees;
  }

  public static boolean isInRightHalfPlane(double signedDegrees) {
    return signedDegrees >= -90 && signedDegrees <= 90;
  }
}
