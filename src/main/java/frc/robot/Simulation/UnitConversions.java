package frc.robot.Simulation;

import edu.wpi.first.math.util.Units;

public class UnitConversions {
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

