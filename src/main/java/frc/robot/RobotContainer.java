// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Auto;
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
  private final Joystick m_joystick1 = new Joystick(Constants.OperatorConstants.kDriverJoystickPort1);
  private final Joystick m_joystick2 = new Joystick(Constants.OperatorConstants.kDriverJoystickPort2);
  private final XboxController m_controller = new XboxController(Constants.OperatorConstants.kDriverControllerPort);

  private final TankDriveSystem m_driveSystem = new TankDriveSystem(
    Constants.OperatorConstants.kLeftMotorForwardChannel,
    Constants.OperatorConstants.kLeftMotorBackChannel,
    Constants.OperatorConstants.kRightMotorBackChannel,
    Constants.OperatorConstants.kRightMotorForwardChannel,
    m_joystick1,
    m_joystick2,
    Constants.OperatorConstants.kSquareInputsDrive,
    Constants.OperatorConstants.kMaxOutput,
    Constants.OperatorConstants.kDeadband,
    Constants.OperatorConstants.kGearBoxRatio,
    Constants.OperatorConstants.kWheelDiameterMeters,
    Constants.OperatorConstants.kUseArcadeDrive
  );

  private final ArmSystem m_armSystem = new ArmSystem(
    Constants.OperatorConstants.kArmWinchChannel,
    Constants.OperatorConstants.kArmExtenderChannel,
    m_controller,
    Constants.OperatorConstants.kDeadband,
    Constants.OperatorConstants.kSquareInputsArm
  );

  private final GrabberSystem m_grabSystem = new GrabberSystem(
    Constants.OperatorConstants.kGrabberForwardChannel,
    Constants.OperatorConstants.kGrabberBackwardChannel,
    m_controller
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
  }

  public Command getAutonomousCommand() {
    return Auto.getAutoCommand(m_driveSystem, m_armSystem, m_grabSystem);
  }
}
