package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSystem extends SubsystemBase{
    private XboxController m_controller;

    private PWMSparkMax m_armWinch;
    private PWMSparkMax m_armExtender;

    public ArmSystem(int armWinchChannel, int armExtenderChannel, XboxController m_controller) {
        m_armWinch = new PWMSparkMax(armWinchChannel);
        m_armExtender = new PWMSparkMax(armExtenderChannel);

        this.m_controller = m_controller;
    }

    public boolean getCondition() {
        return true;
    }

    public CommandBase armCommand() {
        return run(
            () -> {
                m_armWinch.set(m_controller.getLeftY());
                m_armExtender.set(m_controller.getRightY());
            }
        );
    }

    @Override
    public void periodic() {
        m_armWinch.set(m_controller.getLeftY());
        m_armExtender.set(m_controller.getRightY());
    }
    
    @Override
    public void simulationPeriodic() {
        m_armWinch.set(m_controller.getLeftY());
        m_armExtender.set(m_controller.getRightY());
    }
}
