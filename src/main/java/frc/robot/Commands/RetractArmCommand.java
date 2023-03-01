package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ArmSystem;


public class RetractArmCommand extends CommandBase{
    private boolean finished = false;
    ArmSystem m_armSystem;

    public RetractArmCommand(ArmSystem armSystem) {

    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        
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
