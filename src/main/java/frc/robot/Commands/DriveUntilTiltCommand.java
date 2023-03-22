package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.TankDriveSystem;

public class DriveUntilTiltCommand extends CommandBase {
    private TankDriveSystem m_driveSystem;
    private double timeTolerance;
    private double speed;
    private double angleDeadBand;
    private Timer timer;

    public DriveUntilTiltCommand(TankDriveSystem m_driveSystem, double timeTolerance, double angleDeadBand, double speed) {
        this.m_driveSystem = m_driveSystem;
        this.timeTolerance = timeTolerance;
        this.speed = speed;
        addRequirements(m_driveSystem);

        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        m_driveSystem.tankDrive(-speed, -speed, false);
    }

    @Override
    public boolean isFinished() {
        if (timer.get() >= timeTolerance) {
            return true;
        }
        if (MathUtil.applyDeadband(m_driveSystem.getGyroAngle() + 2, angleDeadBand) >= 0) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSystem.tankDrive(0, 0, false);
    }
}
