package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.DrivetrainWrapper.IDrivetrainWrapper;
import frc.robot.DrivetrainWrapper.DrivetrainWrapper;
import frc.robot.RelativeEncoderWrapper.IRelativeEncoderWrapper;

public class TankDriveSystem extends SubsystemBase {
    private Joystick m_controller;
    private boolean squareInputs;

    private IDrivetrainWrapper m_driveTrainWrapper;
    private IRelativeEncoderWrapper m_leftEncoderWrapper;
    private IRelativeEncoderWrapper m_rightEncoderWrapper;

    public TankDriveSystem(int leftMotorBackChannel, int leftMotorForwardChannel, int rightMotorBackChannel,
     int rightMotorForwardChannel, Joystick m_controller, boolean squareInputs, double maxOutput, double Deadband, double gearBoxRatio, double wheelDiameterMeters)
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

        this.m_controller = m_controller;
        this.squareInputs = squareInputs;
    }

    public boolean getCondition() {
        return true;
    }

    public CommandBase driveCommand() {
        return run(
            () -> {
                m_driveTrainWrapper.arcadeDrive(-m_controller.getY(), -m_controller.getX(), squareInputs);
            }
        );
    }

    @Override
    public void periodic() {
        // Simulation components require robotPeriodic to be called
        m_driveTrainWrapper.robotPeriodic();

        m_driveTrainWrapper.arcadeDrive(-m_controller.getY(), -m_controller.getX(), squareInputs);
    }

    @Override
    public void simulationPeriodic() {
      // Simulation components require simulationPeriodic to be called
      m_driveTrainWrapper.simulationPeriodic();

      m_driveTrainWrapper.arcadeDrive(-m_controller.getY(), -m_controller.getX(), squareInputs);
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

    public void tankDrive(double leftSpeed, double rightSpeed, boolean squareInputs) {
        // $TODO Need to quickly passthrough tankDrive, should be easy
        //m_drive.tankDrive(leftSpeed, rightSpeed, squareInputs);
    }
}
