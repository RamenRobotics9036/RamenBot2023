package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSystem;

/**
 * Command to retract the arm fully.
 */
public class RetractArmCommand extends CommandBase {
  ArmSystem m_armSystem;

  public RetractArmCommand(ArmSystem armSystem) {
    m_armSystem = armSystem;
    addRequirements(armSystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_armSystem.setExtenderSpeed(0.1);
  }

  @Override
  public boolean isFinished() {
    return !(m_armSystem.getDigitalSensor());
  }

  @Override
  public void end(boolean interrupted) {
    m_armSystem.setExtenderSpeed(0);
    m_armSystem.resetExtenderEncoder();
    System.out.println("Finished retracting");
  }
}
