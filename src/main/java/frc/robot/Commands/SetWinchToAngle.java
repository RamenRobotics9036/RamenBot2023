package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.ArmSystem;

public class SetWinchToAngle extends CommandBase {
    private ArmSystem m_armSystem;
    private double angle;
    private double speed;
    private int inverse;

    public SetWinchToAngle(ArmSystem m_armSystem, double angle, double speed) {
        this.m_armSystem = m_armSystem;
        addRequirements(m_armSystem);

        this.angle = angle;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        if (m_armSystem.getWinchAbsoluteEncoder() > angle) {
            inverse = -1;
            System.out.println("inversed");
        }
        else {
            inverse = 1;
            System.out.println("not inversed");
        }
        System.out.println("Command initialized with enoder at " +  m_armSystem.getWinchAbsoluteEncoder());
        System.out.println("Command initialized with speed at " +  speed);
    }

    @Override
    public void execute() {
        m_armSystem.setWinchSpeed(inverse*speed);
        System.out.println("Command executed " +  m_armSystem.getWinchAbsoluteEncoder());
    }

    @Override
    public boolean isFinished() {
        if (MathUtil.applyDeadband(m_armSystem.getLeftAxis(), Constants.OperatorConstants.kDeadband) != 0) {
            return true;
            
        }
        System.out.println("ENCODER" + m_armSystem.getWinchAbsoluteEncoder());
        System.out.println("ANGLE" + angle);
        if (inverse == 1 && m_armSystem.getWinchAbsoluteEncoder() >= angle) {
            System.out.println("Is finished with encoder at " + m_armSystem.getWinchAbsoluteEncoder());
            return true;
        } else if (inverse == -1 && m_armSystem.getWinchAbsoluteEncoder() <= angle) {
            System.out.println("Is finished with encoder at " + m_armSystem.getWinchAbsoluteEncoder());
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Ended with encoder at " +  m_armSystem.getWinchAbsoluteEncoder());
        m_armSystem.setWinchSpeed(0);
    }
}
