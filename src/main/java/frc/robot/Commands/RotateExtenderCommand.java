package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSystem;

public class RotateExtenderCommand extends CommandBase {
    private ArmSystem m_armSystem;

    private double speed;
    private double rotations;

    public RotateExtenderCommand(ArmSystem m_armSystem, double speed, double rotations) {
        this.m_armSystem = m_armSystem;
        addRequirements(m_armSystem);

        this.speed = speed;
        this.rotations = rotations;
    }

    @Override
    public void initialize() {
        System.out.println("ROTATE EXTENDER COMMAND INITIALIZED");
    }

    @Override
    public void execute() {
        m_armSystem.setExtenderSpeed(speed);
        System.out.println("EXTENDER MOTOR SPEED SET AT " + m_armSystem.getExtenderSpeed());
        System.out.println("EXTENDER MOTOR ENCODERS SET AT " + m_armSystem.getExtenderEncoder());
    }

    @Override
    public boolean isFinished() {
        if (rotations <= m_armSystem.getExtenderEncoder()) {
            System.out.println("ROTATE EXTENDER COMMAND IS FINISHED SET TO TRUE");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_armSystem.stopSystem();
        System.out.println("ROTATE EXTENDER COMMAND FINISHED");
    }
}
