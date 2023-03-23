package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
        if (MathUtil.applyDeadband(m_driveSystem.getGyroAngle(), 2) > 0) {
            // m_driveSystem.tankDrive(rate, rate, false);
            System.out.println("Speed set to " + rate);
        } if (MathUtil.applyDeadband(m_driveSystem.getGyroAngle(), 2) < 0) {
            // m_driveSystem.tankDrive(-rate, -rate, false);
            System.out.println("Speed set to " + -rate);
        }
    }

    @Override
    public boolean isFinished() {
        if (MathUtil.applyDeadband(m_driveSystem.getGyroAngle(), 2) == 0) {
            System.out.println("Command finished");
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSystem.tankDrive(0, 0, false);
    }
}
