package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.TankDriveSystem;

public class AutoBalanceCommandSlow extends CommandBase {
  private TankDriveSystem m_driveSystem;
  private double rate;
  private Timer timer = new Timer();
  private double cycle = 1;
  private double changeRate;

  public AutoBalanceCommandSlow(TankDriveSystem m_driveSystem, double rate, double changeRate) {
    this.m_driveSystem = m_driveSystem;
    this.rate = rate;
    this.changeRate = changeRate;
    addRequirements(m_driveSystem);
  }

  @Override
  public void initialize() {
    timer.start();
  }

  @Override
  public void execute() {
    double executeRate = rate / cycle;

    if (MathUtil.applyDeadband(m_driveSystem.getGyroRate(), 1) > 0) {
      m_driveSystem.tankDrive(0, 0, false);
      if (timer.get() >= 0.5) {
        cycle += changeRate;
        timer.reset();
      }
    }
    else if (MathUtil.applyDeadband(m_driveSystem.getGyroAngle() + 2.5, 2) > 0) {
      m_driveSystem.tankDrive(executeRate, executeRate, false);
    }
    else if (MathUtil.applyDeadband(m_driveSystem.getGyroAngle() + 2.5, 2) < 0) {
      m_driveSystem.tankDrive(-executeRate, -executeRate, false);
    }
    else {
      m_driveSystem.tankDrive(0, 0, false);
      if (timer.get() >= 0.5) {
        cycle += changeRate; // Changed from 0.5
        timer.reset();
      }
    }

    m_driveSystem.setRate();
    // new WaitCommand(0.5).schedule();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSystem.tankDrive(0, 0, false);
  }
}
