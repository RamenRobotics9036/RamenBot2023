package frc.robot.Commands;

import frc.robot.Subsystems.ArmSystem;

public class ArmExtendFully extends SetExtenderToLength {
  // Constructor
  public ArmExtendFully(ArmSystem armSystem) {
    super(armSystem, -100, 0.9);
  }
}
