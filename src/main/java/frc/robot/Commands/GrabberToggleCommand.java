package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.GrabberSystem;

public class GrabberToggleCommand extends CommandBase{
    private boolean finished = false;
    GrabberSystem m_grabSystem;

    public GrabberToggleCommand(GrabberSystem m_grabSystem) {
        this.m_grabSystem = m_grabSystem;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        m_grabSystem.toggle();
        finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
    
    }
}
