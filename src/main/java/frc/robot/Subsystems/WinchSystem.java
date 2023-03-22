package frc.robot.Subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Commands.RotateMotorCommand;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class WinchSystem extends SubsystemBase{
    private CANSparkMax m_armWinch;
    private RelativeEncoder m_winchEncoder;
    private DutyCycleEncoder m_winchAbsoluteEncoder;
    private double maxOutputWinch;
    private XboxController m_controller;
    private double m_deadband;

    public WinchSystem(int armWinchChannel, double maxOutputWinch, XboxController m_controller, double m_deadband) {
        this.m_deadband = m_deadband;
        this.maxOutputWinch = maxOutputWinch;
        this.m_controller = m_controller; 
        m_armWinch = new CANSparkMax(armWinchChannel, MotorType.kBrushless);
        m_armWinch.setSmartCurrentLimit(20);
        m_winchEncoder = m_armWinch.getEncoder();
        m_winchAbsoluteEncoder = new DutyCycleEncoder(Constants.OperatorConstants.kAbsoluteEncoderWinchChannel);
    }

    public double getWinchAbsoluteEncoder() {
        return m_winchAbsoluteEncoder.getAbsolutePosition();
    }

    private double getWinchAbsoluteEncoderPrivate() {
        return m_winchAbsoluteEncoder.getAbsolutePosition();
    }

    public void setWinchSpeed(double speed) {
        m_armWinch.set(speed);
    }

    public double getLeftAxis() {
        return m_controller.getLeftY();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Absolute encoder", getWinchAbsoluteEncoder());
        double winchOutput = MathUtil.applyDeadband(-m_controller.getLeftY(), m_deadband);
        winchOutput = winchOutput * Math.abs(winchOutput);
        
        if (getWinchAbsoluteEncoderPrivate() >= Constants.OperatorConstants.kWinchEncoderUpperLimit && winchOutput > 0 ) {
            m_armWinch.set(0);
        } else if (getWinchAbsoluteEncoderPrivate() <= Constants.OperatorConstants.kWinchEncoderLowerLimit && winchOutput < 0) {
            m_armWinch.set(0);
        } else {
            setWinchSpeed(winchOutput * maxOutputWinch);
        }
    }
}
