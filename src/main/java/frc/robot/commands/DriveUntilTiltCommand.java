package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.TankDriveSystem;

public class DriveUntilTiltCommand extends CommandBase {
  private TankDriveSystem m_driveSystem;
  private double m_percentOutput;

  public DriveUntilTiltCommand(TankDriveSystem m_driveSystem, double percentOutput) {
    this.m_percentOutput = percentOutput;

    this.m_driveSystem = m_driveSystem;
    addRequirements(m_driveSystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    m_driveSystem.tankDrive(m_percentOutput, m_percentOutput, false);
  }

  @Override
  public boolean isFinished() {
    if (MathUtil.applyDeadband(m_driveSystem.getGyroAngle() + 2, 2.5) > 0) {
      return true;
    }

    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSystem.tankDrive(0, 0, false);
  }
}
