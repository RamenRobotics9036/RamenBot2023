package frc.robot.Commands;

import frc.robot.Subsystems.ArmSystem;

/**
 * Command to extend the arm fully.
 */
public class ArmExtendFully extends SetExtenderToLength {
  // Constructor
  public ArmExtendFully(ArmSystem armSystem) {
    super(armSystem, -100, 0.9);
  }
}
