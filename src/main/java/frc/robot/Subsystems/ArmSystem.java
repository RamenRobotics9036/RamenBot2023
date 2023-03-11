package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SystemConstants;

public class ArmSystem extends SubsystemBase {
    private XboxController m_armController;
    private boolean squareInputs;
    private double deadBandArm;
    private double winchMaxOutput;
    private double extenderMaxOutput;

    private CANSparkMax m_winchMotor;
    private CANSparkMax m_extenderMotor;

    private RelativeEncoder m_winchEncoder;
    private RelativeEncoder m_extenderEncoder;

    public ArmSystem(XboxController m_armController) {
        this.m_armController = m_armController;
        squareInputs = SystemConstants.squareInputsArm;
        deadBandArm = SystemConstants.deadBandArm;
        winchMaxOutput = SystemConstants.winchMaxOutput;
        extenderMaxOutput = SystemConstants.extenderMaxOutput;

        m_winchMotor = new CANSparkMax(RobotConstants.winchMotorChannel, MotorType.kBrushless);
        m_extenderMotor = new CANSparkMax(RobotConstants.extenderMotorChannel, MotorType.kBrushless);

        m_winchEncoder = m_winchMotor.getEncoder();
        m_extenderEncoder = m_extenderMotor.getEncoder();
    }

    public double getWinchEncoder() {
        return m_winchEncoder.getPosition();
    }

    public double getExtenderEncoder() {
        return m_extenderEncoder.getPosition();
    }

    public void setWinchSpeed(double speed) {
        m_winchMotor.set(speed);
    }

    public void setExtenderSpeed(double speed) {
        m_extenderMotor.set(speed);
    }

    public double getWinchSpeed() {
        return m_winchMotor.get();
    }

    public double getExtenderSpeed() {
        return m_extenderMotor.get();
    }

    public void dashBoardInit() {
        SmartDashboard.putNumber("Arm Dead Band", deadBandArm);
        SmartDashboard.putNumber("Winch Max Output", winchMaxOutput);
        SmartDashboard.putNumber("Extender Max Output", extenderMaxOutput);
    }
  
    public void dashBoardUpdate() {
        deadBandArm = SmartDashboard.getNumber("Arm Dead Band", deadBandArm);
        winchMaxOutput = SmartDashboard.getNumber("Winch Max Output", winchMaxOutput);
        extenderMaxOutput = SmartDashboard.getNumber("Extender Max Output", extenderMaxOutput);
        System.out.println("ARM DEAD BAND SET TO " + deadBandArm);
        System.out.println("WINCH MAX OUTPUT SET TO " + winchMaxOutput);
        System.out.println("EXTENDER MAX OUTPUT SET TO " + extenderMaxOutput);

    }

    public CommandBase armCommand() {
        return run(
            () -> {
                if (RobotState.isTeleop()) {
                    double winchSpeed = m_armController.getLeftY();
                    double extenderSpeed = m_armController.getRightY();
    
                    winchSpeed = MathUtil.applyDeadband(winchSpeed, deadBandArm);
                    extenderSpeed = MathUtil.applyDeadband(extenderSpeed, deadBandArm);

                    if (squareInputs) {
                        winchSpeed = winchSpeed * Math.abs(winchSpeed);
                        extenderSpeed = extenderSpeed * Math.abs(extenderSpeed);
                    }

                    m_winchMotor.set(winchSpeed * winchMaxOutput);
                    m_extenderMotor.set(extenderSpeed * extenderMaxOutput);

                    if (m_winchMotor.get() != 0) {
                        System.out.println("WINCH MOTOR SPEED SET TO " + m_winchMotor.get());
                        System.out.println("WINCH ENCODER AT POSITION " + m_winchEncoder.getPosition());
                    }
                    if (m_extenderMotor.get() != 0) {
                        System.out.println("EXTENDER MOTOR SPEED SET TO " + m_extenderMotor.get());
                        System.out.println("EXTENDER ENCODER AT POSITION " + m_extenderEncoder.getPosition());
                    }
                }
            }
        );
    }

    public void stopSystem() {
        m_winchMotor.stopMotor();
        m_extenderMotor.stopMotor();
        System.out.println("ARM SYSTEM STOPPED");
    }
}
