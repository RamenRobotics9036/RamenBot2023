package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ArmSystem;

public class SetExtenderToLength extends CommandBase {
  private ArmSystem m_armSystem;
  private double rotations;
  private double speed;
  private int inverse;

  public SetExtenderToLength(ArmSystem m_armSystem, double rotations, double speed) {
    this.m_armSystem = m_armSystem;
    addRequirements(m_armSystem);

    this.rotations = rotations;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    if (m_armSystem.getExtenderEncoder() > rotations) {
      inverse = -1;
      System.out.println("inversed");
    }
    else {
      inverse = 1;
      System.out.println("not inversed");
    }
  }

  @Override
  public void execute() {
    m_armSystem.setExtenderSpeed(inverse * speed);
  }

  @Override
  public boolean isFinished() {

    if (inverse == 1 && m_armSystem.getExtenderEncoder() >= rotations) {
      return true;
    }
    else if (inverse == -1 && m_armSystem.getExtenderEncoder() <= rotations) {
      return true;
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_armSystem.setExtenderSpeed(0);
  }
}
