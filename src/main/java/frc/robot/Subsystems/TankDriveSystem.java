package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.DrivetrainWrapper.IDrivetrainWrapper;
import frc.robot.DrivetrainWrapper.DrivetrainWrapper;
import frc.robot.RelativeEncoderWrapper.IRelativeEncoderWrapper;

public class TankDriveSystem extends SubsystemBase {
    private Joystick m_controller1;
    private Joystick m_controller2;
    private boolean squareInputs;
    private boolean useArcadeDrive;

    private IDrivetrainWrapper m_driveTrainWrapper;
    private IRelativeEncoderWrapper m_leftEncoderWrapper;
    private IRelativeEncoderWrapper m_rightEncoderWrapper;

    public TankDriveSystem(int leftMotorBackChannel, int leftMotorForwardChannel, int rightMotorBackChannel,
     int rightMotorForwardChannel, Joystick m_controller1, Joystick m_controller2, boolean squareInputs,
    double maxOutput, double Deadband, double gearBoxRatio, double wheelDiameterMeters, boolean useArcadeDrive)
    {
        m_driveTrainWrapper = DrivetrainWrapper.CreateDrivetrainWrapper(
          TimedRobot.isSimulation(),
          leftMotorBackChannel,
          leftMotorForwardChannel,
          rightMotorBackChannel,
          rightMotorForwardChannel,
          gearBoxRatio,
          wheelDiameterMeters);
        
        m_driveTrainWrapper.setMaxOutput(maxOutput);
        m_driveTrainWrapper.setDeadband(Deadband);

        m_leftEncoderWrapper = m_driveTrainWrapper.getLeftEncoder();
        m_rightEncoderWrapper = m_driveTrainWrapper.getRightEncoder();

        this.m_controller1 = m_controller1;
        this.m_controller2 = m_controller2;
        this.squareInputs = squareInputs;
        this.useArcadeDrive = useArcadeDrive;
    }

    public boolean getCondition() {
        return true;
    }

    public CommandBase driveCommand() {
        return run(
            () -> {
                if (useArcadeDrive) {
                    m_driveTrainWrapper.arcadeDrive(-m_controller1.getY(), -m_controller1.getX(), squareInputs);
                } else {
                    m_driveTrainWrapper.tankDrive(-m_controller1.getY(), -m_controller2.getY(), squareInputs);
                }
            }
        );
    }

    @Override
    public void periodic() {
        if (useArcadeDrive) {
            m_driveTrainWrapper.arcadeDrive(-m_controller1.getY(), -m_controller1.getX(), squareInputs);
        } else {
            m_driveTrainWrapper.tankDrive(-m_controller1.getY(), -m_controller2.getY(), squareInputs);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (useArcadeDrive) {
            m_driveTrainWrapper.arcadeDrive(-m_controller1.getY(), -m_controller1.getX(), squareInputs);
        } else {
            m_driveTrainWrapper.tankDrive(-m_controller1.getY(), -m_controller2.getY(), squareInputs);
        }
    }

    public void resetEncoders() {
        m_leftEncoderWrapper.resetPosition();
        m_rightEncoderWrapper.resetPosition();
    }

    public double getLeftEncoder() {
        return m_leftEncoderWrapper.getPosition();
    }

    public double getRightEncoder() {
        return m_rightEncoderWrapper.getPosition();
    }

    public double getAverageEncoderPosition() {
        return (Math.abs(m_leftEncoderWrapper.getPosition()) + Math.abs(m_rightEncoderWrapper.getPosition())) / 2;
    }

    public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
        m_driveTrainWrapper.arcadeDrive(xSpeed, zRotation, squareInputs);
    }

    public void tankDrive(double leftSpeed, double rightSpeed, boolean squareInputs) {
        m_driveTrainWrapper.tankDrive(leftSpeed, rightSpeed, squareInputs);
    }

    public void setDriveMode(boolean mode) {
        useArcadeDrive = mode;
    }
}
