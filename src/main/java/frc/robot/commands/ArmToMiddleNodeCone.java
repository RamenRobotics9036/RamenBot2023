package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.Subsystems.ArmSystem;

/**
 * Raises arm to middle node cone.
 */
public class ArmToMiddleNodeCone extends SetWinchToAngle {
  // Constructor
  public ArmToMiddleNodeCone(ArmSystem armSystem) {
    super(armSystem, Constants.OperatorConstants.kWinchMiddleNodeCone, 1);
  }
}
