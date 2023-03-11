package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSystem;

public class DriveCommand extends CommandBase {
    private DriveSystem m_driveSystem;

    private double speed;
    private double inchesDistance;

    public DriveCommand(DriveSystem m_driveSystem, double speed, double inchesDistance, boolean driveForwards) {
        this.m_driveSystem = m_driveSystem;
        addRequirements(m_driveSystem);

        if (driveForwards) {
            this.speed = speed;
        } else {
            this.speed = -speed;
        }
        this.inchesDistance = inchesDistance;
    }

    @Override
    public void initialize() {
        System.out.println("DRIVE COMMAND INITIALIZED");
    }

    @Override
    public void execute() {
        m_driveSystem.tankDrive(speed, speed);
        System.out.println("DRIVE MOTOR SPEED SET AT " + m_driveSystem.getAverageSpeed());
        System.out.println("DRIVE MOTOR ENCODERS SET AT " + m_driveSystem.getAverageEncoderPosition());
    }

    @Override
    public boolean isFinished() {
        if (inchesDistance <= m_driveSystem.getAverageEncoderPosition() / AutoConstants.gearBoxRatioDrive * AutoConstants.wheelDiameterInchDrive) {
            System.out.println("DRIVE COMMAND IS FINISHED SET TO TRUE");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSystem.stopSystem();
        System.out.println("DRIVE COMMAND ENDED");
    }
}
