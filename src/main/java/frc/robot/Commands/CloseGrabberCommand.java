package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.GrabberSystem;

public class CloseGrabberCommand extends CommandBase {
    private GrabberSystem m_grabber;

    public CloseGrabberCommand(GrabberSystem m_grabber) {
        this.m_grabber = m_grabber;
        addRequirements(m_grabber);
    }

    @Override
    public void initialize() {
        m_grabber.closeGrabber();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
