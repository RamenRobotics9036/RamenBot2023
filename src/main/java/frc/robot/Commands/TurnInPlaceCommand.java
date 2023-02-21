package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.TankDriveSystem;

public class TurnInPlaceCommand extends CommandBase {
    private double rotations;
    private double gearBoxRatio;
    private double percentOutput;

    private boolean turnLeft;
    private boolean turnRight;

    TankDriveSystem m_drive;
    public TurnInPlaceCommand(TankDriveSystem m_drive, double rotations, double gearBoxRatio, double percentOutput, boolean turnLeft, boolean turnRight) {
        this.rotations = rotations;
        this.gearBoxRatio = gearBoxRatio;
        this.percentOutput = percentOutput;

        this.turnLeft = turnLeft;
        this.turnRight = turnRight;

        this.m_drive = m_drive;
        m_drive.resetEncoders();
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (turnLeft) {
            m_drive.arcadeDrive(0, -percentOutput, false);
        } else if (turnRight) {
            m_drive.arcadeDrive(0, percentOutput, false);
        } else {
            throw new Error("You cannot turn in place by turning in both directions or neither directions");
        }
    }

    @Override
    public boolean isFinished() {
        if (rotations <= m_drive.getAverageEncoderPosition() / gearBoxRatio) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.arcadeDrive(0, 0, false);
    }
}
