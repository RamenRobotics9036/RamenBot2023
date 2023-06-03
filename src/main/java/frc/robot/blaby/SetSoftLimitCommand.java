package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ArmSystem;

public class SetSoftLimitCommand extends CommandBase {
  ArmSystem armSystem;

  public SetSoftLimitCommand(ArmSystem armSystem) {
    this.armSystem = armSystem;
    addRequirements(armSystem);
  }

  @Override
  public void initialize() {
    armSystem.setSoftLimit();
  }

  @Override
  public void execute() {
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    armSystem.setExtenderSpeed(0);
  }
}
