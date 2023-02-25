package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.TankDriveSystem;

public class TurnInPlaceCommand extends CommandBase {
    private double rotations;
    private double gearBoxRatio;
    private double percentOutput;
    private double wheelCircumference;

    private boolean turnLeft;
    TankDriveSystem m_drive;

    public TurnInPlaceCommand(TankDriveSystem m_drive, double rotations, double gearBoxRatio, double percentOutput, boolean turnLeft, double wheelCircumference) {
        this.rotations = rotations;
        this.gearBoxRatio = gearBoxRatio;
        this.percentOutput = percentOutput;
        this.wheelCircumference = wheelCircumference;

        this.turnLeft = turnLeft;

        this.m_drive = m_drive;
        m_drive.resetEncoders();
        addRequirements(m_drive);

        // $TODO - Remove this
        System.out.println("Called TurnInPlaceCommand constructor with rotations: " + rotations);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (turnLeft) {
            m_drive.tankDrive(percentOutput, -percentOutput, false);
        } else {
            m_drive.tankDrive(-percentOutput, percentOutput, false);
        }
    }

    @Override
    public boolean isFinished() {
        if (rotations <= m_drive.getAverageEncoderPosition() / gearBoxRatio * wheelCircumference) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.arcadeDrive(0, 0, false);
    }
}
