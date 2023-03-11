package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SystemConstants;

public class DriveSystem extends SubsystemBase {
    private XboxController m_driveController;
    private boolean squareInputs;
    private double maxOutputDrive;
    private double deadBandDrive;

    private CANSparkMax m_frontLeftMotor;
    private CANSparkMax m_backLeftMotor;
    private CANSparkMax m_frontRightMotor;
    private CANSparkMax m_backRightMotor;

    private MotorControllerGroup m_leftMotors;
    private MotorControllerGroup m_rightMotors;
    private DifferentialDrive m_drivetrain;

    private RelativeEncoder m_leftEncoder;
    private RelativeEncoder m_rightEncoder;

    public DriveSystem(XboxController m_driveController) {
        this.m_driveController = m_driveController;
        squareInputs = SystemConstants.squareInputsDrive;
        maxOutputDrive = SystemConstants.maxOutputDrive;
        deadBandDrive = SystemConstants.deadBandDrive;

        m_frontLeftMotor = new CANSparkMax(RobotConstants.frontLeftMotorChannel, MotorType.kBrushless);
        m_backLeftMotor = new CANSparkMax(RobotConstants.backLeftMotorChannel, MotorType.kBrushless);
        m_frontRightMotor = new CANSparkMax(RobotConstants.frontRightMotorChannel, MotorType.kBrushless);
        m_backRightMotor = new CANSparkMax(RobotConstants.backRightMotorChannel, MotorType.kBrushless);

        m_leftMotors = new MotorControllerGroup(m_frontLeftMotor, m_backLeftMotor);
        m_rightMotors = new MotorControllerGroup(m_frontRightMotor, m_backRightMotor);
        m_leftMotors.setInverted(true);

        m_drivetrain = new DifferentialDrive(m_leftMotors, m_rightMotors);
        m_drivetrain.setMaxOutput(maxOutputDrive);

        m_leftEncoder = m_frontLeftMotor.getEncoder();
        m_rightEncoder = m_frontRightMotor.getEncoder();
    }

    public double getLeftEncoder() {
        return m_leftEncoder.getPosition();
    }

    public double getRightEncoder() {
        return m_rightEncoder.getPosition();
    }

    public double getLeftMotorSpeed() {
        return m_leftMotors.get();
    }

    public double getRightMotorSpeed() {
        return m_rightMotors.get();
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        m_drivetrain.tankDrive(leftSpeed, rightSpeed);
    }

    public double getAverageSpeed() {
        return (m_leftMotors.get() + m_rightMotors.get()) / 2;
    }

    public double getAverageEncoderPosition() {
        return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2;
    }

    public void dashBoardInit() {
        SmartDashboard.putNumber("Drive Max Speed", maxOutputDrive);
        SmartDashboard.putNumber("Drive Dead Band", deadBandDrive);
    }
    
      public void dashBoardUpdate() {
        maxOutputDrive = SmartDashboard.getNumber("Drive Max Speed", maxOutputDrive);
        m_drivetrain.setMaxOutput(maxOutputDrive);
        deadBandDrive = SmartDashboard.getNumber("Drive Dead Band", deadBandDrive);
        System.out.println("MAX OUTPUT SET TO " + maxOutputDrive);
        System.out.println("DRIVE DEAD BAND SET TO " + deadBandDrive);
      }

    public CommandBase driveCommand() {
        return run(
            () -> {
                if (RobotState.isTeleop()) {
                    double leftSpeed = m_driveController.getLeftY();
                    double rightSpeed = m_driveController.getRightY();

                    leftSpeed = MathUtil.applyDeadband(leftSpeed, deadBandDrive);
                    rightSpeed = MathUtil.applyDeadband(rightSpeed, deadBandDrive);

                    m_drivetrain.tankDrive(leftSpeed, rightSpeed, squareInputs);

                    if (m_leftMotors.get() != 0) {
                        System.out.println("LEFT MOTOR SPEED SET AT " + m_leftMotors.get());
                        System.out.println("LEFT ENCODER AT POSITION " + m_leftEncoder.getPosition());
                    }
                    if (m_rightMotors.get() != 0) {
                        System.out.println("RIGHT MOTOR SPEED SET AT " + m_rightMotors.get());
                        System.out.println("RIGHT ENCODER AT POSITION " + m_rightEncoder.getPosition());
                    }
                }
            }
        );
    }

    public void stopSystem() {
        m_drivetrain.stopMotor();
        System.out.println("DRIVE SYSTEM STOPPED");
    }
}
