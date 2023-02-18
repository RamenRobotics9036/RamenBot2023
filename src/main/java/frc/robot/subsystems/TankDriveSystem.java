package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDriveSystem extends SubsystemBase {
    private Joystick m_controller;
    private boolean squareInputs;

    private CANSparkMax m_leftFrontMotor;
    private CANSparkMax m_leftBackMotor;
    private CANSparkMax m_rightFrontMotor;
    private CANSparkMax m_rightBackMotor;

    private MotorControllerGroup m_leftGroup;
    private MotorControllerGroup m_rightGroup;
    private DifferentialDrive m_drive;

    private RelativeEncoder m_leftEncoder;
    private RelativeEncoder m_rightEncoder;

    public TankDriveSystem(int leftMotorBackChannel, int leftMotorForwardChannel, int rightMotorBackChannel,
     int rightMotorForwardChannel, Joystick m_controller, boolean squareInputs, double maxOutput)
    {
        m_leftFrontMotor = new CANSparkMax(leftMotorForwardChannel, MotorType.kBrushless);
        m_leftBackMotor = new CANSparkMax(leftMotorBackChannel, MotorType.kBrushless);
        m_rightFrontMotor = new CANSparkMax(rightMotorForwardChannel, MotorType.kBrushless);
        m_rightBackMotor = new CANSparkMax(rightMotorBackChannel, MotorType.kBrushless);

        m_leftGroup = new MotorControllerGroup(m_leftFrontMotor, m_leftBackMotor);
        m_rightGroup = new MotorControllerGroup(m_rightFrontMotor, m_rightBackMotor);
        
        m_drive = new DifferentialDrive(m_leftGroup, m_rightGroup);
        m_drive.setMaxOutput(maxOutput);

        m_leftEncoder = m_leftFrontMotor.getEncoder();
        m_rightEncoder = m_rightFrontMotor.getEncoder();

        this.m_controller = m_controller;
        this.squareInputs = squareInputs;
    }

    public boolean getCondition() {
        return true;
    }

    public CommandBase driveCommand() {
        return run(
            () -> {
                m_drive.arcadeDrive(-m_controller.getY(), -m_controller.getX(), squareInputs);
            }
        );
    }

    @Override
    public void periodic() {
        m_drive.arcadeDrive(-m_controller.getY(), -m_controller.getX(), squareInputs);
    }

    @Override
    public void simulationPeriodic() {
        m_drive.arcadeDrive(-m_controller.getY(), -m_controller.getX(), squareInputs);
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
}
