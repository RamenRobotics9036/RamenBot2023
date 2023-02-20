package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.TankDriveSystem;

public class DriveForwardCommand extends CommandBase{
    private double rotations;
    private double gearBoxRatio;
    private double percentOutput;

    TankDriveSystem m_drive;

    public DriveForwardCommand(TankDriveSystem m_drive, double rotations, double gearBoxRatio, double percentOutput) {
        this.rotations = rotations;
        this.gearBoxRatio = gearBoxRatio;
        this.percentOutput = percentOutput;

        this.m_drive = m_drive;
        m_drive.resetEncoders();
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_drive.arcadeDrive(percentOutput, 0, false);
    }

    @Override
    public boolean isFinished() {
        if (rotations >= m_drive.getAverageEncoderPosition() * gearBoxRatio) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
