package frc.robot.Subsystems;

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

public class ArmSystem extends SubsystemBase {
    private XboxController m_controller;
    private double m_deadband;
    private double maxOutputWinch;
    protected DigitalInput sensor = new DigitalInput(Constants.OperatorConstants.kHallEffectExtenderChannel);

    protected CANSparkMax m_armWinch;
    protected CANSparkMax m_armExtender;

    protected RelativeEncoder m_winchEncoder;
    protected RelativeEncoder m_extenderEncoder;

    protected DutyCycleEncoder m_winchAbsoluteEncoder;

    public ArmSystem(int armWinchChannel, int armExtenderChannel, XboxController m_controller, double m_deadband,
            boolean squareInputs, double maxOutputWinch) {
        m_armWinch = new CANSparkMax(armWinchChannel, MotorType.kBrushless);
        m_armWinch.setSmartCurrentLimit(20); // $TODO - For simulation, test that smart limits actually work when I set
                                             // a value on SparkMax
        m_armExtender = new CANSparkMax(armExtenderChannel, MotorType.kBrushless);
        m_armExtender.setSmartCurrentLimit(20); // $TODO - For simulation, test that smart limits actually work when I
                                                // set a value on SparkMax
        m_armExtender.setInverted(false);
        m_winchEncoder = m_armWinch.getEncoder();
        m_extenderEncoder = m_armExtender.getEncoder();

        this.m_controller = m_controller;
        this.m_deadband = m_deadband;
        this.maxOutputWinch = maxOutputWinch;

        m_winchAbsoluteEncoder = new DutyCycleEncoder(Constants.OperatorConstants.kAbsoluteEncoderWinchChannel);

        setSoftLimit();
    }

    public void initDashBoard() {
        SmartDashboard.putNumber("Winch Encoder", m_winchEncoder.getPosition());
        SmartDashboard.putNumber("Extender Encoder", m_extenderEncoder.getPosition());
        SmartDashboard.putNumber("Winch Max Output", maxOutputWinch);
    }

    public void updateDashBoard() {
        SmartDashboard.putNumber("Winch Encoder", m_winchEncoder.getPosition());
        SmartDashboard.putNumber("Extender Encoder", m_extenderEncoder.getPosition());
        maxOutputWinch = SmartDashboard.getNumber("Winch Max Output", maxOutputWinch);
    }

    public double getWinchAbsoluteEncoder() {
        return m_winchAbsoluteEncoder.getAbsolutePosition();
    }

    private double getWinchAbsoluteEncoderPrivate() {
        return m_winchAbsoluteEncoder.getAbsolutePosition();
    }

    public void putSensorOutputs() {
        SmartDashboard.putNumber("Winch Absolute Encoder Position", m_winchAbsoluteEncoder.getAbsolutePosition());
        SmartDashboard.putBoolean("Hall Effect Sensor Output", getDigitalSensor());
    }

    public boolean getCondition() {
        return true;
    }

    public boolean isOffLower() {
        return getWinchAbsoluteEncoderPrivate() <= Constants.OperatorConstants.kWinchEncoderLowerLimit;
    }

    public boolean isOffHigher() {
        return getWinchAbsoluteEncoderPrivate() >= Constants.OperatorConstants.kWinchEncoderUpperLimit;
    }

    public CommandBase getDefaultArmCommand() {
        CommandBase defaultCommand = run(
                () -> {
                    //System.out.println("ARMCOMMAND");
                    ProcessJoystickInputForArm();
                });

        defaultCommand.setName("Default");
        return defaultCommand;
    }

    private void ProcessJoystickInputForArm() {
        // SmartDashboard.putNumber("Absolute encoder", getWinchAbsoluteEncoder());
        // SmartDashboard.putNumber("Extender encoder", getExtenderEncoder());
        double winchOutput = MathUtil.applyDeadband(-m_controller.getLeftY(), m_deadband);
        double extenderOutput = MathUtil.applyDeadband(m_controller.getRightY(), m_deadband);
        winchOutput = winchOutput * Math.abs(winchOutput);
        extenderOutput = extenderOutput * Math.abs(extenderOutput);

        if (getWinchAbsoluteEncoderPrivate() != 0.0) {
            if (getWinchAbsoluteEncoderPrivate() >= Constants.OperatorConstants.kWinchEncoderUpperLimit
                    && winchOutput > 0) {
                m_armWinch.set(0);
            } else if (getWinchAbsoluteEncoderPrivate() <= Constants.OperatorConstants.kWinchEncoderLowerLimit
                    && winchOutput < 0) {
                m_armWinch.set(0);
            } else {
                setWinchSpeed(winchOutput * maxOutputWinch);
            }
        } else {
            setWinchSpeed(winchOutput * maxOutputWinch);
        }

        // $TODO - For simulation, test that smart limits actually work when I set a value on SparkMax
        if (getExtenderEncoder() <= Constants.OperatorConstants.kExtenderSoftLimitTurns && extenderOutput < 0) { 
            m_armExtender.set(0);
        } else if (getExtenderEncoder() > 0 && extenderOutput > 0) {
            m_armExtender.set(0);
        } else {
            setExtenderSpeed(extenderOutput);
        }
    }

    @Override
    public void periodic() {
        Double winchAbsoluteEncoder = Double.valueOf(getWinchAbsoluteEncoderPrivate());
        // SmartDashboard.putNumber("Winch Absolute Position",
        // getWinchAbsoluteEncoderPrivate());
        SmartDashboard.putBoolean("Winch Absolute Encoder", !(winchAbsoluteEncoder == 0.0));
    }

    @Override
    public void simulationPeriodic() {
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

    public Double getWinchEncoder() {
        return m_winchEncoder.getPosition();
    }

    public Double getExtenderEncoder() {
        return m_extenderEncoder.getPosition();
    }

    public RotateMotorCommand rotateWinchMotor(double rotations, double gearBoxRatio, double percentOutput,
            double winchCircumference) {
        return new RotateMotorCommand(m_armWinch, m_extenderEncoder, rotations, gearBoxRatio, percentOutput,
                winchCircumference);
    }

    public RotateMotorCommand rotateExtenderMotor(double rotations, double gearBoxRatio, double percentOutput,
            double extenderCircumference) {
        return new RotateMotorCommand(m_armExtender, m_extenderEncoder, rotations, gearBoxRatio, percentOutput,
                extenderCircumference);
    }

    public void setWinchSpeed(double speed) {
        m_armWinch.set(speed);
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
        // $TODO - For simulation, test that smart limits actually work when I set a value on SparkMax
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
