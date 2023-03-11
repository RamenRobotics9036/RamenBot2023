package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabSystem;

public class GrabCommand extends CommandBase {
    private GrabSystem m_grabSystem;
    private Value value;
    private boolean toggle;
    
    public GrabCommand(GrabSystem m_grabSystem, Value value) {
        this.m_grabSystem = m_grabSystem;
        this.value = value;
        toggle = false;
    }

    public GrabCommand(GrabSystem m_grabSystem) {
        this.m_grabSystem = m_grabSystem;
        value = m_grabSystem.getValue();
        toggle = true;
    }

    @Override
    public void initialize() {
        System.out.println("GRAB COMMAND INITIALIZED");
        if (toggle) {
            m_grabSystem.toggleSolenoid();
            System.out.println("TOGGLED GRABBED SYSTEM");
        } else {
            m_grabSystem.setSolenoid(value);
        }
        System.out.println("GRAB COMMAND SET SOLENOID VALUE TO " + m_grabSystem.getValue());
    }

    @Override
    public void execute() {
        System.out.println("GRAB COMMAND EXECUTED");
    }

    @Override
    public boolean isFinished() {
        System.out.println("GRAB COMMAND IS FINISHED SET TO TRUE");
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        m_grabSystem.stopSystem();
        System.out.println("GRAB COMMAND FINISHED");
    }
}
