package frc.robot.Subsystems;

import javax.swing.border.EtchedBorder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Commands.RotateMotorCommand;

public class ArmSystem extends SubsystemBase{
    private XboxController m_controller;
    private double m_deadband;
    private DigitalInput sensor = new DigitalInput(Constants.OperatorConstants.kHallEffectExtenderChannel);

    private CANSparkMax m_armExtender;

    private RelativeEncoder m_extenderEncoder;

    private DutyCycleEncoder m_winchAbsoluteEncoder;

    public ArmSystem(int armExtenderChannel, XboxController m_controller, double m_deadband, boolean squareInputs, double maxOutputWinch) {

        m_armExtender = new CANSparkMax(armExtenderChannel, MotorType.kBrushless);
        m_armExtender.setSmartCurrentLimit(20);
        m_armExtender.setInverted(false);
        m_extenderEncoder = m_armExtender.getEncoder();

        this.m_controller = m_controller;
        this.m_deadband = m_deadband;

        initDashBoard();
        setSoftLimit();
    }

    private void initDashBoard() {
        SmartDashboard.putNumber("Extender Encoder", m_extenderEncoder.getPosition());
    }

    public void updateDashBoard() {
        SmartDashboard.putNumber("Extender Encoder", m_extenderEncoder.getPosition());
    }


    public void putSensorOutputs() {
        SmartDashboard.putBoolean("Hall Effect Sensor Output", getDigitalSensor());
    }
 
    public boolean getCondition() {
        return true;
    }


    public CommandBase armCommand() {
        return run(
            () -> {
                // SmartDashboard.putNumber("Num rotations", m_extenderEncoder.getPosition());
                double extenderOutput = MathUtil.applyDeadband(m_controller.getRightY(), m_deadband);
                extenderOutput = extenderOutput * Math.abs(extenderOutput);

                setExtenderSpeed(extenderOutput);
            }
        );
    }

    

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Extender encoder", getExtenderEncoder());
        double extenderOutput = MathUtil.applyDeadband(m_controller.getRightY(), m_deadband);
        extenderOutput = extenderOutput * Math.abs(extenderOutput);

        SmartDashboard.putNumber("Extender output", extenderOutput);

        if (getExtenderEncoder() <= Constants.OperatorConstants.kExtenderSoftLimitTurns && extenderOutput < 0) {
            m_armExtender.set(0);
        } else if (getExtenderEncoder() > 0 && extenderOutput > 0) {
            m_armExtender.set(0);
        } else {
            setExtenderSpeed(extenderOutput);
        }

        // setExtenderSpeed(extenderOutput);
    }


    
    @Override
    public void simulationPeriodic() {

    }

    public void resetEncoders() {
        m_extenderEncoder.setPosition(0);
    }

    public void resetExtenderEncoder() {
        m_extenderEncoder.setPosition(0);
    }

    public double getExtenderEncoder() {
        return m_extenderEncoder.getPosition();
    }

    public void setExtenderSpeed(double speed) {
        m_armExtender.set(speed);
    }

    public double getLeftAxis() {
        return m_controller.getLeftY();
    }
    public boolean getDigitalSensor() {
        return sensor.get();
    }

    public void setSoftLimit() {
        m_armExtender.enableSoftLimit(SoftLimitDirection.kForward, false);
        m_armExtender.enableSoftLimit(SoftLimitDirection.kReverse, false);
        resetExtenderEncoder();
    }

    public double getArmCurrent() {
        return m_armExtender.getOutputCurrent();
    }

    public void setArmCurrent(int value) {
        m_armExtender.setSmartCurrentLimit(value);
    }
}

