package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDriveSystem;

/**
 * This class represents a command for an automated balance control system for tank drive.
 */
public class AutoBalanceCommandSlow extends CommandBase {
  private TankDriveSystem m_driveSystem;
  private double m_rate;
  private Timer m_timer = new Timer();
  private double m_cycle = 1;
  private double m_changeRate;

  /**
   * Constructor.
   * 
   */
  public AutoBalanceCommandSlow(TankDriveSystem driveSystem, double rate, double changeRate) {
    this.m_driveSystem = driveSystem;
    this.m_rate = rate;
    this.m_changeRate = changeRate;
    addRequirements(driveSystem);
  }

  @Override
  public void initialize() {
    m_timer.start();
  }

  @Override
  public void execute() {
    double executeRate = m_rate / m_cycle;

    if (MathUtil.applyDeadband(m_driveSystem.getGyroRate(), 1) > 0) {
      m_driveSystem.tankDrive(0, 0, false);
      if (m_timer.get() >= 0.5) {
        m_cycle += m_changeRate;
        m_timer.reset();
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
      if (m_timer.get() >= 0.5) {
        m_cycle += m_changeRate; // Changed from 0.5
        m_timer.reset();
      }
    }

    m_driveSystem.setRate();
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
