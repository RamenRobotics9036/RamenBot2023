package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSystem;

public class RotateWinchCommand extends CommandBase {
    private ArmSystem m_armSystem;

    private double speed;
    private double rotations;

    public RotateWinchCommand(ArmSystem m_armSystem, double speed, double rotations) {
        this.m_armSystem = m_armSystem;
        addRequirements(m_armSystem);

        this.speed = speed;
        this.rotations = rotations;
    }

    @Override
    public void initialize() {
        System.out.println("ROTATE WINCH COMMAND INITIALIZED");
    }

    @Override
    public void execute() {
        m_armSystem.setWinchSpeed(speed);
        System.out.println("WINCH MOTOR SPEED SET AT " + m_armSystem.getWinchSpeed());
        System.out.println("WINCH MOTOR ENCODERS SET AT " + m_armSystem.getWinchEncoder());
    }

    @Override
    public boolean isFinished() {
        if (rotations <= m_armSystem.getWinchEncoder()) {
            System.out.println("ROTATE WINCH COMMAND IS FINISHED SET TO TRUE");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_armSystem.stopSystem();
        System.out.println("ROTATE WINCH COMMAND FINISHED");
    }
}
