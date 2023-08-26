package frc.robot.Commands;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnActiveIntake extends CommandBase {
  private MotorController m_motor1;
  private MotorController m_motor2;
  private double speed;

  public TurnActiveIntake(MotorController m_motor1, MotorController m_motor2, double speed) {
      this.m_motor1 = m_motor1;
      this.m_motor2 = m_motor2;
      this.speed = speed;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void schedule() {
      m_motor1.set(speed);
      if (m_motor2 != null) {
          m_motor2.set(speed);
      }
  }

  @Override
  public void end(boolean interrupted) {
      m_motor1.set(0);
      if (m_motor2 != null) {
          m_motor2.set(0);
      }
  }
}
