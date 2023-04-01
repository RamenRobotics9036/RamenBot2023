package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.TankDriveSystem;

public class TurnInPlaceCommand extends CommandBase {
    private double rotations; 
    private double gearBoxRatio;
    private double percentOutput;
    private double wheelCircumference;
    private Timer time;

    private boolean turnLeft;
    TankDriveSystem m_drive;

    public TurnInPlaceCommand(TankDriveSystem m_drive, double rotations, double gearBoxRatio, double percentOutput, boolean turnLeft, double wheelCircumference) {
        this.rotations = rotations;
        this.gearBoxRatio = gearBoxRatio;
        this.percentOutput = percentOutput;
        this.wheelCircumference = wheelCircumference;

        this.turnLeft = turnLeft;
        time = new Timer();

        this.m_drive = m_drive;
        m_drive.resetEncoders();
        addRequirements(m_drive);
        
    }

    @Override
    public void initialize() {
        time.start();
    }

    @Override
    public void execute() {
        m_drive.tankDrive(percentOutput, -percentOutput, true);
        System.out.println("Turned");
    }

    @Override
    public boolean isFinished() {


        // m_drive.getAverageEncoderPosition() / gearBoxRatio * wheelCircumference
        // This code ^: if motor spins once, wheel will spin 1/8.28 times * circumfrence = distance traveled <= this is wrong?
        // Check encoders if # of actual rotations is greather than or equal to # of wanted rotations
        if (time.get() >= 0.4) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.tankDrive(0, 0, false);
    }
}
