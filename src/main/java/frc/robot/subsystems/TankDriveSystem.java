package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDriveSystem extends SubsystemBase {
    private Joystick m_controller;
    private boolean squareInputs;

    private PWMSparkMax m_leftFrontMotor;
    private PWMSparkMax m_leftBackMotor;
    private PWMSparkMax m_rightFrontMotor;
    private PWMSparkMax m_rightBackMotor;

    private MotorControllerGroup m_leftGroup;
    private MotorControllerGroup m_rightGroup;
    private DifferentialDrive m_drive;

    public TankDriveSystem(int leftMotorBackChannel, int leftMotorForwardChannel, int rightMotorBackChannel,
     int rightMotorForwardChannel, Joystick m_controller, boolean squareInputs)
    {
        m_leftFrontMotor = new PWMSparkMax(leftMotorForwardChannel);
        m_leftBackMotor = new PWMSparkMax(leftMotorBackChannel);
        m_rightFrontMotor = new PWMSparkMax(rightMotorForwardChannel);
        m_rightBackMotor = new PWMSparkMax(rightMotorBackChannel);

        m_leftGroup = new MotorControllerGroup(m_leftFrontMotor, m_leftBackMotor);
        m_rightGroup = new MotorControllerGroup(m_rightFrontMotor, m_rightBackMotor);
        m_drive = new DifferentialDrive(m_leftGroup, m_rightGroup);

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
}
