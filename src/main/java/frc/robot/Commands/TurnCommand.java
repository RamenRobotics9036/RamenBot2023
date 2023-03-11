package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSystem;

public class TurnCommand extends CommandBase {
    private DriveSystem m_driveSystem;

    private double speed;
    private double inchesDistance;
    private boolean turnLeft;

    public TurnCommand(DriveSystem m_driveSystem, double speed, double inchesDistance, boolean turnLeft) {
        this.m_driveSystem = m_driveSystem;
        addRequirements(m_driveSystem);

        this.speed = speed;
        this.inchesDistance = inchesDistance;
        this.turnLeft = turnLeft;
    }

    @Override
    public void initialize() {
        System.out.println("TURN COMMAND INITIALIZED");
    }

    @Override
    public void execute() {
        if (turnLeft) {
            m_driveSystem.tankDrive(speed, -speed);
            System.out.println("LEFT MOTOR SPEED SET AT " + m_driveSystem.getLeftMotorSpeed());
            System.out.println("LEFT ENCODER AT POSITION " + m_driveSystem.getLeftEncoder());
        } else {
            m_driveSystem.tankDrive(-speed, speed);
            System.out.println("RIGHT MOTOR SPEED SET AT " + m_driveSystem.getRightMotorSpeed());
            System.out.println("RIGHT ENCODER AT POSITION " + m_driveSystem.getRightEncoder());
        }
    }

    @Override
    public boolean isFinished() {
        if (turnLeft) {
            if (inchesDistance <= m_driveSystem.getLeftEncoder() / AutoConstants.gearBoxRatioDrive * AutoConstants.wheelDiameterInchDrive) {
                System.out.println("TURN COMMAND IS FINISHED SET TO TRUE");
                return true;
            }
        } else {
            if (inchesDistance <= m_driveSystem.getRightEncoder() / AutoConstants.gearBoxRatioDrive * AutoConstants.wheelDiameterInchDrive) {
                System.out.println("TURN COMMAND IS FINISHED SET TO TRUE");
                return true;
            }
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSystem.stopSystem();
        System.out.println("TURN COMMAND ENDED");
    }
}
