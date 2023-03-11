// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.GrabSystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private XboxController m_driveController = new XboxController(Constants.ControllerConstants.driveControllerPort);
  private XboxController m_armController = new XboxController(Constants.ControllerConstants.armControllerPort);

  private DriveSystem m_driveSystem = new DriveSystem(m_driveController);
  private ArmSystem m_armSystem = new ArmSystem(m_armController);
  private GrabSystem m_grabSystem = new GrabSystem(m_armController);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
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
  private void configureBindings() {
    m_driveSystem.setDefaultCommand(m_driveSystem.driveCommand());
    m_armSystem.setDefaultCommand(m_armSystem.armCommand());
    m_grabSystem.setDefaultCommand(m_grabSystem.grabCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Autos.getAuto(m_driveSystem, m_armSystem, m_grabSystem);
  }

  public void dashBoardInit() {
    Autos.dashBoardInit();
    m_driveSystem.dashBoardInit();
    m_armSystem.dashBoardInit();
    m_grabSystem.dashBoardInit();
  }

  public void dashBoardUpdate() {
    m_driveSystem.dashBoardUpdate();
    m_armSystem.dashBoardUpdate();
    m_grabSystem.dashBoardUpdate();
  }
}
