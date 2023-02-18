package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSystem extends SubsystemBase{
    private XboxController m_controller;

    private CANSparkMax m_armWinch;
    private CANSparkMax m_armExtender;

    private RelativeEncoder m_winchEncoder;
    private RelativeEncoder m_extenderEncoder;

    public ArmSystem(int armWinchChannel, int armExtenderChannel, XboxController m_controller) {
        m_armWinch = new CANSparkMax(armWinchChannel, MotorType.kBrushless);
        m_armExtender = new CANSparkMax(armExtenderChannel, MotorType.kBrushless);

        m_winchEncoder = m_armWinch.getEncoder();
        m_extenderEncoder = m_armExtender.getEncoder();

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

    public void resetEncoders() {
        m_winchEncoder.setPosition(0);
        m_extenderEncoder.setPosition(0);
    }

    public void resetWinchEncoder() {
        m_winchEncoder.setPosition(0);
    }

    public void resetExtenderEncoder() {
        m_extenderEncoder.setPosition(0);
    }

    public double getWinchEncoder() {
        return m_winchEncoder.getPosition();
    }

    public double getExtenderEncoder() {
        return m_extenderEncoder.getPosition();
    }
}
