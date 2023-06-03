package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.Subsystems.ArmSystem;

/**
 * Lowers arm all the way down to the ground.
 */
public class ArmToGround extends SetWinchToAngle {
  // Constructor
  public ArmToGround(ArmSystem armSystem) {
    super(armSystem, Constants.OperatorConstants.kWinchGroundAngle, 1);
  }
}
