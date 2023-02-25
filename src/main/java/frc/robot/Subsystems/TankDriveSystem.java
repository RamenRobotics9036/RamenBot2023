package frc.robot.Subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.DrivetrainWrapper.IDrivetrainWrapper;
import frc.robot.DrivetrainWrapper.DrivetrainWrapper;
import frc.robot.RelativeEncoderWrapper.IRelativeEncoderWrapper;

public class TankDriveSystem extends SubsystemBase {
    private Joystick m_controller1;
    private Joystick m_controller2;

    private boolean squareInputs;
    private boolean useArcadeDrive;
    private double maxOutput;

    private SlewRateLimiter slewLimiter;
    private double slewLimit;

    private IDrivetrainWrapper m_driveTrainWrapper;
    private IRelativeEncoderWrapper m_leftEncoderWrapper;
    private IRelativeEncoderWrapper m_rightEncoderWrapper;

    public TankDriveSystem(int leftMotorBackChannel, int leftMotorForwardChannel, int rightMotorBackChannel,
     int rightMotorForwardChannel, Joystick m_controller1, Joystick m_controller2, boolean squareInputs,
    double maxOutput, double Deadband, double gearBoxRatio, double wheelDiameterMeters, boolean useArcadeDrive,
    double slewLimit)
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
        this.maxOutput = maxOutput;

        this.slewLimit = SmartDashboard.getNumber("Slew Limit", slewLimit);
        if (this.slewLimit > 0) {
            slewLimiter = new SlewRateLimiter(slewLimit);
        }

        initDashBoard();
    }

    private void initDashBoard() {
        SmartDashboard.putNumber("Max Speed", maxOutput);
        SmartDashboard.putBoolean("Drive Mode", useArcadeDrive);
        SmartDashboard.putNumber("Left Encoder", m_leftEncoderWrapper.getPosition());
        SmartDashboard.putNumber("Right Encoder", m_rightEncoderWrapper.getPosition());
        SmartDashboard.putNumber("Slew Limit", slewLimit);
    }

    private void updateDashBoard() {
        useArcadeDrive = SmartDashboard.getBoolean("Drive Mode", useArcadeDrive);
        maxOutput = SmartDashboard.getNumber("Max Speed", maxOutput);
        m_driveTrainWrapper.setMaxOutput(maxOutput);
        SmartDashboard.putNumber("Left Encoder", m_leftEncoderWrapper.getPosition());
        SmartDashboard.putNumber("Right Encoder", m_rightEncoderWrapper.getPosition());
    }

    // Simulated drivetrain requires periodic calls to function
    private void updatePeriodicSimulation(boolean simulationPeriodic) {
      if (simulationPeriodic) {
        m_driveTrainWrapper.simulationPeriodic();
      } else {
        m_driveTrainWrapper.robotPeriodic();
      }

      // $TODO - Remove this
      // System.out.println("Called updatePeriodicSimulation (simulationPeriodic: " + simulationPeriodic + ")");
    }

    public boolean getCondition() {
        return true;
    }

    // $TODO - Is this being called?
    public CommandBase driveCommand() {
        return run(
            () -> {
                updateDashBoard();
                if (useArcadeDrive) {
                    m_driveTrainWrapper.arcadeDrive(slewLimiter.calculate(-m_controller1.getY()), -m_controller1.getX(), squareInputs);
                } else {
                    m_driveTrainWrapper.tankDrive(slewLimiter.calculate(-m_controller1.getY()), slewLimiter.calculate(-m_controller2.getY()), squareInputs);
                }
            }
        );
    }

    @Override
    public void periodic() {
        updateDashBoard();
        updatePeriodicSimulation(false);

        // $TODO - This shouldn't be called
        //if (useArcadeDrive) {
        //    m_driveTrainWrapper.arcadeDrive(slewLimiter.calculate(-m_controller1.getY()), -m_controller1.getX(), squareInputs);
        //} else {
        //    m_driveTrainWrapper.tankDrive(slewLimiter.calculate(-m_controller1.getY()), slewLimiter.calculate(-m_controller2.getY()), squareInputs);
        //}
    }

    @Override
    public void simulationPeriodic() {
        updateDashBoard();
        updatePeriodicSimulation(true);

        // $TODO - This shouldn't be called
        //if (useArcadeDrive) {
        //    m_driveTrainWrapper.arcadeDrive(slewLimiter.calculate(-m_controller1.getY()), -m_controller1.getX(), squareInputs);
        //} else {
        //    m_driveTrainWrapper.tankDrive(slewLimiter.calculate(-m_controller1.getY()), slewLimiter.calculate(-m_controller2.getY()), squareInputs);
        //}
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
        // $TODO - Remove this
        System.out.println("TankDriveSystem::tankDrive called leftSpeed: " + leftSpeed + ", rightSpeed: " + rightSpeed);
    
        m_driveTrainWrapper.tankDrive(leftSpeed, rightSpeed, squareInputs);
    }
}
