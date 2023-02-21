package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSystem extends SubsystemBase {
    private XboxController m_controller;
    private final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);
    private PneumaticHub m_pneumaticHub;
    private DoubleSolenoid m_solenoid;

    public GrabberSystem(int grabberForwardChannel, int grabberBackwardChannel, XboxController m_controller) {
        m_pneumaticHub = new PneumaticHub();
        m_compressor.enableDigital();
        m_solenoid = m_pneumaticHub.makeDoubleSolenoid(grabberForwardChannel, grabberBackwardChannel);

        this.m_controller = m_controller;
    }

    public boolean getCondition() {
        return true;
    }

    public CommandBase grabCommand() {
        return run(
            () -> {
                if (m_controller.getRightBumperReleased()) {
                    m_solenoid.set(Value.kForward);
                } else if (m_controller.getLeftBumperReleased()) {
                    m_solenoid.set(Value.kReverse);
                }
            }
        );
    }

    @Override
    public void periodic() {
        if (m_controller.getRightBumperReleased()) {
            m_solenoid.set(Value.kForward);
        } else if (m_controller.getLeftBumperReleased()) {
            m_solenoid.set(Value.kReverse);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (m_controller.getRightBumperReleased()) {
            m_solenoid.set(Value.kForward);
        } else if (m_controller.getLeftBumperReleased()) {
            m_solenoid.set(Value.kReverse);
        }
    }
}