package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.TankDriveSystem;

public class TurnDegrees extends CommandBase {
  private double m_percentOutput;
  private double m_degrees;
  private double m_initialHeading;
  private TankDriveSystem m_drive;

  public TurnDegrees(TankDriveSystem m_drive, double percentOutput, double degrees) {
    this.m_percentOutput = percentOutput;
    this.m_degrees = degrees;
    this.m_drive = m_drive;

    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    this.m_initialHeading = m_drive.getGyroYaw();
  }

  @Override
  public void execute() {
    // Calculate the error
    double error = m_degrees - (m_drive.getGyroYaw() - m_initialHeading);

    // Wrap error to be within -180 to 180 degrees
    error = ((error + 180) % 360) - 180;
    if (error < -180) {
      error += 360;
    }

    double direction = Math.signum(error);
    m_drive.tankDrive(-1 * direction * m_percentOutput, direction * m_percentOutput, true);
  }

  @Override
  public boolean isFinished() {
    // Check if the robot is within an acceptable error range (e.g., 2 degrees)
    return Math.abs(m_degrees - (m_drive.getGyroYaw() - m_initialHeading)) < 2;
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.tankDrive(0, 0, false);
  }
}
