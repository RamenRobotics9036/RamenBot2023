package frc.robot.Commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotateMotorCommand extends CommandBase {
  private double distance;
  private double gearBoxRatio;
  private double percentOutput;
  private double wheelCircumference;
  private Timer time;

  private MotorController m_motor;
  private RelativeEncoder m_encoder;

  public RotateMotorCommand(MotorController m_motor,
      RelativeEncoder m_encoder,
      double distance,
      double gearBoxRatio,
      double percentOutput,
      double wheelCircumference) {
    this.distance = distance;
    this.gearBoxRatio = gearBoxRatio;
    this.percentOutput = percentOutput;
    this.wheelCircumference = wheelCircumference;

    this.m_motor = m_motor;
    this.m_encoder = m_encoder;
    time = new Timer();
  }

  @Override
  public void initialize() {
    time.start();
  }

  @Override
  public void execute() {
    m_motor.set(percentOutput);
  }

  @Override
  public boolean isFinished() {
    if (time.get() > 0.5) {
      return true;
    }
    // System.out.println("Enc pos" + m_encoder.getPosition());
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_motor.stopMotor();
  }
}
