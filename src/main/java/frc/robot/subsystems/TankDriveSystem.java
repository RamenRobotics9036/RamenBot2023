package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.filter.SlewRateLimiter;
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
  private SlewRateLimiter turboLimiter1;
  private SlewRateLimiter turboLimiter2;

  private double slewLimit;
  private double turboSlew;

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
  private double m_prevRate = 0;

  protected double m_wheelDiameterMeters;
  protected double m_gearBoxRatio;

  // Constructor
  public TankDriveSystem(int leftMotorBackChannel,
      int leftMotorForwardChannel,
      int rightMotorBackChannel,
      int rightMotorForwardChannel,
      XboxController controller,
      boolean squareInputs,
      double maxOutput,
      double Deadband,
      double gearBoxRatio,
      double wheelDiameterMeters,
      double slewLimit,
      double turboSlew) {
    m_gearBoxRatio = gearBoxRatio;
    m_wheelDiameterMeters = wheelDiameterMeters;

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

    this.m_controller = controller;

    this.squareInputs = squareInputs;
    this.maxOutput = maxOutput;

    this.slewLimit = slewLimit;
    this.turboSlew = turboSlew;
    if (this.slewLimit > 0) {
      slewLimiter1 = new SlewRateLimiter(slewLimit);
      slewLimiter2 = new SlewRateLimiter(slewLimit);
    }

    turboLimiter1 = new SlewRateLimiter(turboSlew);
    turboLimiter2 = new SlewRateLimiter(turboSlew);

    initDashBoard();
  }

  private void initDashBoard() {
    // SmartDashboard.putNumber("Max Speed", maxOutput);
    // SmartDashboard.putNumber("Left Encoder", m_leftEncoder.getPosition());
    // SmartDashboard.putNumber("Right Encoder", m_rightEncoder.getPosition());
    // SmartDashboard.putNumber("Slew Limit", slewLimit);
  }

  public void updateDashBoard() {
    // maxOutput = SmartDashboard.getNumber("Max Speed", maxOutput);
    // m_drive.setMaxOutput(maxOutput);
    // SmartDashboard.putNumber("Left Encoder", m_leftEncoder.getPosition());
    // SmartDashboard.putNumber("Right Encoder", m_rightEncoder.getPosition());
  }

  public boolean getCondition() {
    return true;
  }

  public CommandBase getDefaultDriveCommand() {
    return run(() -> {
      // System.out.println("DRIVECOMMAND");
      ProcessJoystickInput();
    });
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
    return (Math.abs(getLeftEncoder()) + Math.abs(getRightEncoder())) / 2;
  }

  private void ProcessJoystickInput() {
    // In autonomous, the default command is to tell the robot to STOP. Recall
    // that we have to send a voltage-level to the motors EVERY 20ms, otherwise
    // the watchdog trips
    double leftAxisY = RobotState.isAutonomous() ? 0 : m_controller.getLeftY();
    double rightAxisY = RobotState.isAutonomous() ? 0 : m_controller.getRightY();

    double xSpeed = (leftAxisY + rightAxisY) / 2;
    double zRotation = (leftAxisY - rightAxisY) / 2;

    if (m_controller.getRightTriggerAxis() > Constants.OperatorConstants.kDeadband) {
      m_drive.setMaxOutput(1);
      double slewLimit1 = turboLimiter1.calculate(xSpeed);
      double slewLimit2 = turboLimiter2.calculate(zRotation);

      arcadeDrive(slewLimit1,
          slewLimit2 * Constants.OperatorConstants.kRotationDilation,
          squareInputs);
      slewLimiter1.reset(slewLimit1);
      slewLimiter2.reset(slewLimit2);
    }
    else {

      m_drive.setMaxOutput(maxOutput);
      double slewLimit1 = slewLimiter1.calculate(xSpeed);
      double slewLimit2 = slewLimiter2.calculate(zRotation);

      arcadeDrive(slewLimit1,
          slewLimit2 * Constants.OperatorConstants.kRotationDilation,
          squareInputs);
      turboLimiter1.reset(slewLimit1);
      turboLimiter2.reset(slewLimit2);
    }
    SmartDashboard.putNumber("Left Motor", m_leftMotors.get());
    SmartDashboard.putNumber("Right Motor", m_rightMotors.get());

    // if (m_controller.getRightTriggerAxis() > 0) {
    // m_drive.tankDrive(1, 0, false);
    // }
    // if (m_controller.getLeftTriggerAxis() > 0) {
    // m_drive.tankDrive(0, 1, false);
    // }
  }

  public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
    m_drive.arcadeDrive(xSpeed, zRotation, squareInputs);
  }

  public void tankDrive(double leftSpeed, double rightSpeed, boolean squareInputs) {
    m_drive.tankDrive(leftSpeed, rightSpeed, squareInputs);
  }

  public void calibrate() {
  }

  public double getGyroAngle() {
    return m_gyro.getRoll();
  }

  public double getGyroYaw() {
    return m_gyro.getYaw();
  }

  public double getGyroRate() {
    return Math.abs(m_prevRate - m_gyro.getRoll());
  }

  public void setRate() {
    m_prevRate = m_gyro.getRoll();
  }
}
