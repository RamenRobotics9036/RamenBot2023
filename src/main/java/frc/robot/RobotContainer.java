// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Auto;
import frc.robot.Commands.SetWinchToAngle;
import frc.robot.Commands.SetSoftLimitCommand;
import frc.robot.Commands.RetractArmCommand;
import frc.robot.Subsystems.ArmSystem;
import frc.robot.Subsystems.GrabberSystem;
import frc.robot.Subsystems.TankDriveSystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final XboxController m_controller1 = new XboxController(Constants.OperatorConstants.kDriverControllerPort1);
  private final XboxController m_controller2 = new XboxController(Constants.OperatorConstants.kDriverControllerPort2);

  public final TankDriveSystem m_driveSystem = new TankDriveSystem(
    Constants.OperatorConstants.kLeftMotorForwardChannel,
    Constants.OperatorConstants.kLeftMotorBackChannel,
    Constants.OperatorConstants.kRightMotorBackChannel,
    Constants.OperatorConstants.kRightMotorForwardChannel,
    m_controller1,
    Constants.OperatorConstants.kSquareInputsDrive,
    Constants.OperatorConstants.kMaxOutputDrive,
    Constants.OperatorConstants.kDeadband,
    Constants.OperatorConstants.kGearBoxRatioDrive,
    Constants.OperatorConstants.kWheelDiameterMetersDrive,
    Constants.OperatorConstants.kSlewLimit
  );

  public final ArmSystem m_armSystem = new ArmSystem(
    Constants.OperatorConstants.kArmWinchChannel,
    Constants.OperatorConstants.kArmExtenderChannel,
    m_controller2,
    Constants.OperatorConstants.kDeadband,
    Constants.OperatorConstants.kSquareInputsArm,
    Constants.OperatorConstants.kMaxOutputWinch
  );

  public final GrabberSystem m_grabSystem = new GrabberSystem(
    Constants.OperatorConstants.kGrabberForwardChannel,
    Constants.OperatorConstants.kGrabberBackwardChannel,
    m_controller2
    );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // CameraServer.startAutomaticCapture();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  public void configureBindings() {
    new Trigger(m_driveSystem::getCondition).whileTrue(m_driveSystem.driveCommand());
    new Trigger(m_armSystem::getCondition).whileTrue(m_armSystem.armCommand());
    new Trigger(m_grabSystem::getCondition).whileTrue(m_grabSystem.grabCommand());

    new Trigger(m_controller2::getAButtonReleased).onTrue(new SetWinchToAngle(m_armSystem, Constants.OperatorConstants.kWinchMiddleNodeCone, 0.5));
    new Trigger(m_controller2::getXButtonReleased).onTrue(new SetWinchToAngle(m_armSystem, Constants.OperatorConstants.kWinchMiddleNodeCube, 0.5));
    new Trigger(m_controller2::getBButtonReleased).onTrue(new SetWinchToAngle(m_armSystem, Constants.OperatorConstants.kWinchGroundAngle, 0.5));
    // new Trigger(m_controller2::getYButtonReleased).onTrue(new SetWinchToAngle(m_armSystem, Constants.OperatorConstants.kWinchRetractAngle, 0.5));

    new Trigger(m_controller2::getYButtonReleased).onTrue(new RetractArmCommand(m_armSystem).andThen(new SetSoftLimitCommand(m_armSystem)));

    // new Trigger(m_armSystem::isOffHigher).onTrue(new SetWinchToAngle(m_armSystem, Constants.OperatorConstants.kEmergencyAngle, 0.5));
  }

  public Command getAutonomousCommand() {
    System.out.println("Auto command scheduled container");
    return Auto.getAutoCommand(m_driveSystem, m_armSystem, m_grabSystem);
  }

  public void putShuffleBoardAutoCommands() {
    Auto.putShuffleBoardCommands(m_driveSystem, m_armSystem, m_grabSystem);
    // m_armSystem.putSensorOutputs();
  }

  public void resetEncoders() {
    m_driveSystem.resetEncoders();
    m_armSystem.resetEncoders();
  }

  public void initDashboard() {
    SmartDashboard.putNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed);
    SmartDashboard.putNumber("Auto Motor Distance", Constants.OperatorConstants.kAutoMotorDistance);
    SmartDashboard.putBoolean("Auto Turn Left", true);
    SmartDashboard.putNumber("Starting Position", 0);
  }

  public void updateDashBoard() {
    m_driveSystem.updateDashBoard();
    m_armSystem.updateDashBoard();
  }
}
