package frc.robot.DrivetrainWrapper;

import frc.robot.RelativeEncoderWrapper.IRelativeEncoderWrapper;
import frc.robot.RelativeEncoderWrapper.RelativeEncoderWrapperSimulation;

public class DrivetrainWrapperSimulation implements IDrivetrainWrapper {

  private Drivetrain m_driveTrain;
  private IRelativeEncoderWrapper m_leftEncoderWrapper;
  private IRelativeEncoderWrapper m_rightEncoderWrapper;

  // Constructor
  public DrivetrainWrapperSimulation(double physicalGearBoxRatio, double physicalWheelDiameterMeters) {
    m_driveTrain = new Drivetrain();

    m_leftEncoderWrapper = new RelativeEncoderWrapperSimulation(m_driveTrain.getLeftEncoder(), physicalGearBoxRatio, physicalWheelDiameterMeters);
    m_rightEncoderWrapper = new RelativeEncoderWrapperSimulation(m_driveTrain.getRightEncoder(), physicalGearBoxRatio, physicalWheelDiameterMeters);

    m_leftEncoderWrapper.resetPosition();
    m_rightEncoderWrapper.resetPosition();
  }

  public void setMaxOutput(double maxOutput) {
    m_driveTrain.setMaxOutput(maxOutput);
  }

  public void setDeadband(double deadband) {
    m_driveTrain.setDeadband(deadband);
  }
  
  public void robotPeriodic() {
    m_driveTrain.periodic();
  }

  public void simulationPeriodic() {
    m_driveTrain.simulationPeriodic();
  }

  public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
    m_driveTrain.arcadeDrive(xSpeed, zRotation, squareInputs);
  }

  public void tankDrive(double leftSpeed, double rightSpeed, boolean squareInputs) {
    // $TODO - Remove this
    System.out.println("leftSpeed + rightSpeed / 2: " + (leftSpeed + rightSpeed) / 2 + ", rightSpeed - leftSpeed / 2: " + (rightSpeed - leftSpeed) / 2);

    m_driveTrain.arcadeDrive((leftSpeed + rightSpeed) / 2, (rightSpeed - leftSpeed) / 2, squareInputs);
  }

  public IRelativeEncoderWrapper getLeftEncoder() {
    return m_leftEncoderWrapper;
  }

  public IRelativeEncoderWrapper getRightEncoder() {
    return m_rightEncoderWrapper;
  }
}

