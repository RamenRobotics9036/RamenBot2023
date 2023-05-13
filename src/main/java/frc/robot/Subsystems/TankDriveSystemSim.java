package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Simulation.DriveSimulation;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TankDriveSystemSim extends TankDriveSystem {
  private DriveSimulation m_driveSimulation = null;

  public static TankDriveSystem CreateTankDriveSystemInstance(int leftMotorBackChannel, int leftMotorForwardChannel,
      int rightMotorBackChannel,
      int rightMotorForwardChannel, XboxController controller, boolean squareInputs,
      double maxOutput, double Deadband, double gearBoxRatio, double wheelDiameterMeters,
      double slewLimit, double turboSlew) {
    TankDriveSystem result;

    if (RobotBase.isSimulation()) {
      result = new TankDriveSystemSim(
          Constants.OperatorConstants.kLeftMotorBackChannel,
          Constants.OperatorConstants.kLeftMotorForwardChannel,
          Constants.OperatorConstants.kRightMotorBackChannel,
          Constants.OperatorConstants.kRightMotorForwardChannel,
          controller,
          Constants.OperatorConstants.kSquareInputsDrive,
          Constants.OperatorConstants.kMaxOutputDrive,
          Constants.OperatorConstants.kDeadband,
          Constants.OperatorConstants.kGearBoxRatioDrive,
          Constants.OperatorConstants.kWheelDiameterMetersDrive,
          Constants.OperatorConstants.kSlewLimit,
          Constants.OperatorConstants.kTurboSlew);

      System.out.println("TANKDRIVESYSTEM: **** Simulation ****");

    } else {
      result = new TankDriveSystem(
          Constants.OperatorConstants.kLeftMotorBackChannel,
          Constants.OperatorConstants.kLeftMotorForwardChannel,
          Constants.OperatorConstants.kRightMotorBackChannel,
          Constants.OperatorConstants.kRightMotorForwardChannel,
          controller,
          Constants.OperatorConstants.kSquareInputsDrive,
          Constants.OperatorConstants.kMaxOutputDrive,
          Constants.OperatorConstants.kDeadband,
          Constants.OperatorConstants.kGearBoxRatioDrive,
          Constants.OperatorConstants.kWheelDiameterMetersDrive,
          Constants.OperatorConstants.kSlewLimit,
          Constants.OperatorConstants.kTurboSlew);

      System.out.println("TANKDRIVESYSTEM: Physical Robot version");
    }

    return result;
  }

  // Constructor
  public TankDriveSystemSim(int leftMotorBackChannel, int leftMotorForwardChannel, int rightMotorBackChannel,
      int rightMotorForwardChannel, XboxController controller, boolean squareInputs,
      double maxOutput, double Deadband, double gearBoxRatio, double wheelDiameterMeters,
      double slewLimit, double turboSlew) {
    // FIRST, we call superclass
    super(leftMotorBackChannel,
        leftMotorForwardChannel,
        rightMotorBackChannel,
        rightMotorForwardChannel,
        controller,
        squareInputs,
        maxOutput,
        Deadband,
        gearBoxRatio,
        wheelDiameterMeters,
        slewLimit,
        turboSlew);

    // This entire class should only be instantiated when we're under simulation.
    // But just in-case someone tries to instantiate it otherwise, we do an extra
    // check here.
    if (RobotBase.isSimulation()) {
      m_driveSimulation = new DriveSimulation(wheelDiameterMeters / 2);
      resetSimulationRobotPosition();
    }
  }

  private boolean isRobotEnabled() {
    return RobotState.isEnabled();
  }

  @Override
  public void periodic() {
    super.periodic();

    // When Robot is disabled, the entire simulation freezes
    if (isRobotEnabled()) {
      m_driveSimulation.periodic();
    }
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    // When Robot is disabled, the entire simulation freezes
    if (isRobotEnabled()) {
      m_driveSimulation.simulationPeriodic();
    }
  }

  @Override
  public void resetEncoders() {
    super.resetEncoders();

    m_driveSimulation.resetRelativeEncoders();
  }

  @Override
  public double getGyroYaw() {
    return m_driveSimulation.getHeading();
  }

  private void resetSimulationRobotPosition() {
    Pose2d initialPosition = new Pose2d(2, 2, new Rotation2d());
    m_driveSimulation.resetOdometry(initialPosition);
  }

  // RETURN SIMULATED VALUE: Overrides physical encoder value in parent class
  @Override
  public double getLeftEncoder() {
    // Note that our relativeEncoder returns distance the SIMULATED robot moved on
    // the field in meters.
    // But we want to return number of MOTOR rotations that our PHYSICAL robot would
    // have had to take to move that distance in real life.
    return m_driveSimulation.getRelativeDistanceLeft() * m_gearBoxRatio / (m_wheelDiameterMeters * Math.PI);
  }

  // RETURN SIMULATED VALUE: Overrides physical encoder value in parent class
  @Override
  public double getRightEncoder() {
    // Note that our relativeEncoder returns distance the SIMULATED robot moved on
    // the field in meters.
    // But we want to return number of MOTOR rotations that our PHYSICAL robot would
    // have had to take to move that distance in real life.
    return m_driveSimulation.getRelativeDistanceRight() * m_gearBoxRatio / (m_wheelDiameterMeters * Math.PI);
  }

  @Override
  public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
    super.arcadeDrive(xSpeed, zRotation, squareInputs);

    // When Robot is disabled, the entire simulation freezes
    if (isRobotEnabled()) {
      m_driveSimulation.arcadeDrive(xSpeed, zRotation, squareInputs);
    }
  }

  @Override
  public void tankDrive(double leftSpeed, double rightSpeed, boolean squareInputs) {
    super.tankDrive(leftSpeed, rightSpeed, squareInputs);

    // When Robot is disabled, the entire simulation freezes
    if (isRobotEnabled()) {
      m_driveSimulation.tankDrive(leftSpeed, rightSpeed, squareInputs);
    }
  }
}
