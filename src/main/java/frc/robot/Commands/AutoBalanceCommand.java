package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.TankDriveSystem;

public class AutoBalanceCommand extends CommandBase{
    private boolean finished = false;
    private TankDriveSystem m_driveSystem;
    private double rate;

    public AutoBalanceCommand(TankDriveSystem m_driveSystem, double rate) {
        this.m_driveSystem = m_driveSystem;
        this.rate = rate;
        addRequirements(m_driveSystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if (MathUtil.applyDeadband(m_driveSystem.getGyroAngle() + 2, 2) > 0) {
            m_driveSystem.tankDrive(rate, rate, false);
        } if (MathUtil.applyDeadband(m_driveSystem.getGyroAngle() + 2, 2) < 0) {
            m_driveSystem.tankDrive(-rate, -rate, false);
        }
        new WaitCommand(0.5);
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
