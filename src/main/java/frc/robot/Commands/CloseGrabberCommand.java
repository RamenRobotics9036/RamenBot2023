package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.GrabberSystem;

public class CloseGrabberCommand extends CommandBase {
  private boolean finished = false;
  private GrabberSystem m_grabber;

  public CloseGrabberCommand(GrabberSystem grabber) {
    this.m_grabber = grabber;
    addRequirements(m_grabber);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_grabber.closeGrabber();
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
