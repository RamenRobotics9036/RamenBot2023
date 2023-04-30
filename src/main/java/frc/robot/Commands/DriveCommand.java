package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.TankDriveSystem;

public class DriveCommand extends CommandBase{
    private double distance;
    private double gearBoxRatio;
    private double percentOutput;
    private double wheelCircumference;

    TankDriveSystem m_drive;

    public DriveCommand(TankDriveSystem m_drive, double distance, double gearBoxRatio, double percentOutput, double wheelCircumference) {
        this.distance = distance;
        this.gearBoxRatio = gearBoxRatio;
        this.percentOutput = -percentOutput;
        this.wheelCircumference = wheelCircumference;

        this.m_drive = m_drive;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_drive.resetEncoders();
    }

    @Override
    public void execute() {
        m_drive.tankDrive(-percentOutput, -percentOutput, false);
    }

    @Override
    public boolean isFinished() {
        if (distance <= m_drive.getAverageEncoderPosition() / gearBoxRatio * wheelCircumference) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.tankDrive(0, 0, false);
    }
}
