package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.TankDriveSystem;

public class TurnDegrees extends CommandBase {
    private double percentOutput;
    private double degrees;
    private double initialHeading;

    TankDriveSystem m_drive;

    public TurnDegrees(TankDriveSystem m_drive, double percentOutput, double degrees) {
        this.percentOutput = percentOutput;
        this.degrees = degrees;
        this.m_drive = m_drive;
        
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
      this.initialHeading = m_drive.getGyroYaw();
    }

    @Override
    public void execute() {
        // Calculate the error
        double error = degrees - (m_drive.getGyroYaw() - initialHeading);
        
        // Wrap error to be within -180 to 180 degrees
        error = ((error + 180) % 360) - 180;
        if (error < -180) {
            error += 360;
        }

        double direction = Math.signum(error);
        m_drive.tankDrive(-1 * direction * percentOutput, direction * percentOutput, true);
    }

    @Override
    public boolean isFinished() {
        // Check if the robot is within an acceptable error range (e.g., 2 degrees)
        return Math.abs(degrees - (m_drive.getGyroYaw() - initialHeading)) < 2;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.tankDrive(0, 0, false);
    }
}

