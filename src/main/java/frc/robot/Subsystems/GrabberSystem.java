package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSystem extends SubsystemBase {
    private XboxController m_controller;
    private final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);
    private PneumaticHub m_pneumaticHub;
    protected DoubleSolenoid m_solenoid;

    public GrabberSystem(int grabberForwardChannel, int grabberBackwardChannel, XboxController controller) {
        m_pneumaticHub = new PneumaticHub();
        m_compressor.enableDigital();
        m_solenoid = m_pneumaticHub.makeDoubleSolenoid(grabberForwardChannel, grabberBackwardChannel);

        this.m_controller = controller;
    }

    public boolean getCondition() {
        return true;
    }

    public CommandBase grabCommand() {
        return run(
            () -> {
            if (!RobotState.isAutonomous()) {
                if (m_controller.getLeftBumperReleased()) {
                    m_solenoid.set(Value.kForward);
                } else if (m_controller.getRightBumperReleased()) {
                    m_solenoid.set(Value.kReverse);
                }
            }
            }
        );
    }

    @Override
    public void periodic() {
        if (!RobotState.isAutonomous()) {
            if (m_controller.getLeftBumperReleased()) {
                m_solenoid.set(Value.kForward);
            } else if (m_controller.getRightBumperReleased()) {
                m_solenoid.set(Value.kReverse);
            }
        }
    }

    @Override
    public void simulationPeriodic() {
    }

    public void toggle() {
        m_solenoid.toggle();
    }
    public void openGrabber(){
        if (RobotState.isAutonomous()) {
            m_solenoid.set(Value.kForward);
        }
    }

    public void closeGrabber(){
        if (RobotState.isAutonomous()) {
            m_solenoid.set(Value.kReverse);
        }
    }
}