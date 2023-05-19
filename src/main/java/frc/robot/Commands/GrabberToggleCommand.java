package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.GrabberSystem;

public class GrabberToggleCommand extends CommandBase {
    private boolean finished = false;
    GrabberSystem m_grabSystem;

    public GrabberToggleCommand(GrabberSystem grabSystem) {
        this.m_grabSystem = grabSystem;
        addRequirements(m_grabSystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_grabSystem.openGrabber();
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
