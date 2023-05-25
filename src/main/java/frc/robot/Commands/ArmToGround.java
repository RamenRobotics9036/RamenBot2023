package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.Subsystems.ArmSystem;

public class ArmToGround extends SetWinchToAngle {
  // Constructor
  public ArmToGround(ArmSystem armSystem) {
    super(armSystem, Constants.OperatorConstants.kWinchGroundAngle, 1);
  }
}
