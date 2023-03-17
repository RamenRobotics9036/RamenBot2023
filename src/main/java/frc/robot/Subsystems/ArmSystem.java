package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.RotateMotorCommand;
import frc.robot.Commands.SetArmPositionCommand;

public class ArmSystem extends SubsystemBase{
    private XboxController m_controller;
    private double m_deadband;
    private double maxOutputWinch;

    private CANSparkMax m_armWinch;
    private CANSparkMax m_armExtender;

    private RelativeEncoder m_winchEncoder;
    private RelativeEncoder m_extenderEncoder;

    public ArmSystem(int armWinchChannel, int armExtenderChannel, XboxController m_controller, double m_deadband, boolean squareInputs, double maxOutputWinch) {
        m_armWinch = new CANSparkMax(armWinchChannel, MotorType.kBrushless);
        m_armExtender = new CANSparkMax(armExtenderChannel, MotorType.kBrushless);
        m_armExtender.setInverted(true);

        m_winchEncoder = m_armWinch.getEncoder();
        m_extenderEncoder = m_armExtender.getEncoder();

        this.m_controller = m_controller;
        this.m_deadband = m_deadband;
        this.maxOutputWinch = maxOutputWinch;

        initDashBoard();
    }

    private void initDashBoard() {
        SmartDashboard.putNumber("Winch Encoder", m_winchEncoder.getPosition());
        SmartDashboard.putNumber("Extender Encoder", m_extenderEncoder.getPosition());
        SmartDashboard.putNumber("Winch Max Output", maxOutputWinch);
    }

    public void updateDashBoard() {
        SmartDashboard.putNumber("Winch Encoder", m_winchEncoder.getPosition());
        SmartDashboard.putNumber("Extender Encoder", m_extenderEncoder.getPosition());
        maxOutputWinch = SmartDashboard.getNumber("Winch Max Output", maxOutputWinch);
    }
 
    public boolean getCondition() {
        return true;
    }

    public CommandBase armCommand() {
        return run(
            () -> {
                if (m_controller.getAButtonPressed()){
                    new SetArmPositionCommand(0, m_winchEncoder , m_armWinch);
                } else if (m_controller.getBButtonPressed()){
                    new SetArmPositionCommand(3, m_winchEncoder , m_armWinch);
                }
                double winchOutput = MathUtil.applyDeadband(-m_controller.getLeftY(), m_deadband);
                double extenderOutput = MathUtil.applyDeadband(m_controller.getRightY(), m_deadband);
                winchOutput = winchOutput * Math.abs(winchOutput);
                extenderOutput = extenderOutput * Math.abs(extenderOutput);
        
                m_armWinch.set(winchOutput * maxOutputWinch);
                m_armExtender.set(extenderOutput);
            }
        );
    }

    @Override
    public void periodic() {
        double winchOutput = MathUtil.applyDeadband(-m_controller.getLeftY(), m_deadband);
        double extenderOutput = MathUtil.applyDeadband(m_controller.getRightY(), m_deadband);
        winchOutput = winchOutput * Math.abs(winchOutput);
        extenderOutput = extenderOutput * Math.abs(extenderOutput);

        m_armWinch.set(winchOutput * maxOutputWinch);
        m_armExtender.set(extenderOutput);
    }
    
    @Override
    public void simulationPeriodic() {
        double winchOutput = MathUtil.applyDeadband(m_controller.getLeftY(), m_deadband);
        double extenderOutput = MathUtil.applyDeadband(m_controller.getRightY(), m_deadband);
        winchOutput = winchOutput * Math.abs(winchOutput);
        extenderOutput = extenderOutput * Math.abs(extenderOutput);

        m_armWinch.set(winchOutput * maxOutputWinch);
        m_armExtender.set(extenderOutput);
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

    public RotateMotorCommand rotateWinchMotor(double rotations, double gearBoxRatio, double percentOutput, double winchCircumference) {
        return new RotateMotorCommand(m_armWinch, m_extenderEncoder, rotations, gearBoxRatio, percentOutput, winchCircumference);
    }

    public RotateMotorCommand rotateExtenderMotor(double rotations, double gearBoxRatio, double percentOutput, double extenderCircumference) {
        return new RotateMotorCommand(m_armExtender, m_extenderEncoder, rotations, gearBoxRatio, percentOutput, extenderCircumference);
    }

    public void setWinchSpeed(double speed) {
        m_armWinch.set(speed);
    }

    public void setExtenderSpeed(double speed) {
        m_armExtender.set(speed);
    }
}
