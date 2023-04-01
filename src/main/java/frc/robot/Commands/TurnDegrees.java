package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.TankDriveSystem;

public class TurnDegrees extends CommandBase {
    private double percentOutput;
    private double degrees;
    private double tolerance;

    TankDriveSystem m_drive;

    public TurnDegrees(TankDriveSystem m_drive, double percentOutput, double degrees) {
        this.percentOutput = percentOutput;
        this.degrees = degrees;
        this.m_drive = m_drive;
        this.tolerance = m_drive.getGyroYaw();
        
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_drive.tankDrive(percentOutput, -percentOutput, true);
        // SmartDashboard.putNumber("Gyro Yaw", MathUtil.applyDeadband(m_drive.getGyroYaw() + 2.5 - tolerance, 2));
    }

    @Override
    public boolean isFinished() {
           if (MathUtil.applyDeadband(m_drive.getGyroYaw() + 2.5 - tolerance, 2) > 180) {
                return true;
           }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.tankDrive(0, 0, false);
    }
}
