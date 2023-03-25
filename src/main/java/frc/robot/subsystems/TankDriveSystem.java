package frc.robot.Subsystems;


import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TankDriveSystem extends SubsystemBase {
    private XboxController m_controller;

    private boolean squareInputs;
    private double maxOutput;

    private SlewRateLimiter slewLimiter1;
    private SlewRateLimiter slewLimiter2;
    private double slewLimit;

    private CANSparkMax m_leftMotor1;
    private CANSparkMax m_leftMotor2;
    private CANSparkMax m_rightMotor1;
    private CANSparkMax m_rightMotor2;

    private MotorControllerGroup m_leftMotors;
    private MotorControllerGroup m_rightMotors;

    private DifferentialDrive m_drive;
    private RelativeEncoder m_leftEncoder;
    private RelativeEncoder m_rightEncoder;

    private final Pigeon2 m_gyro = new Pigeon2(7);

    public TankDriveSystem(int leftMotorBackChannel, int leftMotorForwardChannel, int rightMotorBackChannel,
     int rightMotorForwardChannel, XboxController m_controller, boolean squareInputs,
    double maxOutput, double Deadband, double gearBoxRatio, double wheelDiameterMeters,
    double slewLimit)
    {
        m_leftMotor1 = new CANSparkMax(leftMotorBackChannel, MotorType.kBrushless);
        m_leftMotor2 = new CANSparkMax(leftMotorForwardChannel, MotorType.kBrushless);
        m_rightMotor1 = new CANSparkMax(rightMotorForwardChannel, MotorType.kBrushless);
        m_rightMotor2 = new CANSparkMax(rightMotorBackChannel, MotorType.kBrushless);

        m_leftMotors = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);
        m_rightMotors = new MotorControllerGroup(m_rightMotor1, m_rightMotor2);
        m_rightMotors.setInverted(true);
        m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
        
        m_drive.setMaxOutput(maxOutput);
        m_drive.setDeadband(Deadband);

        m_leftEncoder = m_leftMotor1.getEncoder();
        m_rightEncoder = m_rightMotor1.getEncoder();

        this.m_controller = m_controller;

        this.squareInputs = squareInputs;
        this.maxOutput = maxOutput;

        this.slewLimit = SmartDashboard.getNumber("Slew Limit", slewLimit);
        if (this.slewLimit > 0) {
            slewLimiter1 = new SlewRateLimiter(slewLimit);
            slewLimiter2 = new SlewRateLimiter(slewLimit);
        }

        initDashBoard();
    }

    private void initDashBoard() {
        SmartDashboard.putNumber("Max Speed", maxOutput);
        SmartDashboard.putNumber("Left Encoder", m_leftEncoder.getPosition());
        SmartDashboard.putNumber("Right Encoder", m_rightEncoder.getPosition());
        SmartDashboard.putNumber("Slew Limit", slewLimit);
    }

    public void updateDashBoard() {
        maxOutput = SmartDashboard.getNumber("Max Speed", maxOutput);
        m_drive.setMaxOutput(maxOutput);
        SmartDashboard.putNumber("Left Encoder", m_leftEncoder.getPosition());
        SmartDashboard.putNumber("Right Encoder", m_rightEncoder.getPosition());
    }

    public boolean getCondition() {
        return true;
    }

    public CommandBase driveCommand() {
        return run(
            () -> {
                double leftAxis = m_controller.getLeftY();
                double rightAxis = m_controller.getRightY();
                double xForward = (leftAxis + rightAxis) / 2;
                double zRotation = (leftAxis - rightAxis) / 2;

                if (Constants.OperatorConstants.kUseArcadeDrive == false){
                    m_drive.arcadeDrive(slewLimiter1.calculate(xForward), slewLimiter2.calculate(zRotation) * Constants.OperatorConstants.kRotationDilation, squareInputs);
                }
                System.out.println("Command called");
            }
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro Roll ", m_gyro.getRoll() + 2);
        SmartDashboard.putNumber("Gyro Deadband ", MathUtil.applyDeadband(m_gyro.getRoll() + 2, 2));

        if (!RobotState.isAutonomous()) {
            double leftAxis = m_controller.getLeftY();
            double rightAxis = m_controller.getRightY();
            double xForward = (leftAxis + rightAxis) / 2;
            double zRotation = (leftAxis - rightAxis) / 2;

            if (Constants.OperatorConstants.kUseArcadeDrive == false){
                m_drive.arcadeDrive(slewLimiter1.calculate(xForward), slewLimiter2.calculate(zRotation) * Constants.OperatorConstants.kRotationDilation, squareInputs);
            }
        }
    }



    @Override
    public void simulationPeriodic() {
        double leftAxis = m_controller.getLeftY();
        double rightAxis = m_controller.getRightY(); 
        double xForward = (leftAxis + rightAxis) / 2;
        double zRotation = (leftAxis - rightAxis) / 2;

        if (Constants.OperatorConstants.kUseArcadeDrive == false){
            m_drive.arcadeDrive(slewLimiter1.calculate(xForward),
            slewLimiter2.calculate(zRotation) * Constants.OperatorConstants.kRotationDilation, squareInputs);
        }
    }



    public void resetEncoders() {
        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
    }

    public double getLeftEncoder() {
        return m_leftEncoder.getPosition();
    } 

    public double getRightEncoder() {
        return m_rightEncoder.getPosition();
    }

    public double getAverageEncoderPosition() {
        return (Math.abs(m_leftEncoder.getPosition()) + Math.abs(m_rightEncoder.getPosition())) / 2;
    }

    public void tankDrive(double leftSpeed, double rightSpeed, boolean squareInputs) {
        m_drive.tankDrive(leftSpeed, rightSpeed, squareInputs);
    }

    public void calibrate() {
    }

    public double getGyroAngle() {
        return m_gyro.getRoll();
    }
}
