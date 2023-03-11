package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class GrabSystem extends SubsystemBase {
    private XboxController m_grabController;

    private final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);
    private PneumaticHub m_pneumaticHub;
    private DoubleSolenoid m_solenoid;

    public GrabSystem(XboxController m_grabController) {
        this.m_grabController = m_grabController;

        m_pneumaticHub = new PneumaticHub();
        m_compressor.enableDigital();
        m_solenoid = m_pneumaticHub.makeDoubleSolenoid(RobotConstants.solenoidForwardChannel, RobotConstants.solenoidForwardChannel);
    }

    public void setSolenoid(Value value) {
        m_solenoid.set(value);
    }

    public void toggleSolenoid() {
        m_solenoid.toggle();
    }

    public Value getValue() {
        return m_solenoid.get();
    }

    public void dashBoardInit() {
    }
    
    public void dashBoardUpdate() {
    }

    public CommandBase grabCommand() {
        return run(
            () -> {
                if (RobotState.isTeleop()) {
                    if (m_grabController.getLeftBumperReleased()) {
                        m_solenoid.set(Value.kForward);
                        System.out.println("SOLENOID POSITION SET AT " + m_solenoid.get());
                    } else if (m_grabController.getRightBumperReleased()) {
                        m_solenoid.set(Value.kReverse);
                        System.out.println("SOLENOID POSITION SET AT " + m_solenoid.get());
                    }
                }
            }
        );
    }

    public void stopSystem() {
        System.out.println("GRAB SYSTEM STOPPED");
    }
}
