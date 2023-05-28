package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ArmSystem;

public class RetractArmCommand extends CommandBase {
  ArmSystem armSystem;

  public RetractArmCommand(ArmSystem armSystem) {
    this.armSystem = armSystem;
    addRequirements(armSystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    armSystem.setExtenderSpeed(0.7);
  }

  @Override
  public boolean isFinished() {
    if (!armSystem.getDigitalSensor()) {
      return true;
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    armSystem.setExtenderSpeed(0);
    armSystem.resetExtenderEncoder();
  }
}
