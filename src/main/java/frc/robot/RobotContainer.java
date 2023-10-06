// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Auto;
import frc.robot.commands.RetractArmCommand;
import frc.robot.commands.SetSoftLimitCommand;
import frc.robot.commands.SetWinchToAngle;
import frc.robot.commands.TurnActiveIntake;
import frc.robot.subsystems.ActiveIntakeSystem;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.ArmSystemSim;
import frc.robot.subsystems.GrabberSystem;
import frc.robot.subsystems.GrabberSystemSim;
import frc.robot.subsystems.TankDriveSystem;
import frc.robot.subsystems.TankDriveSystemSim;
import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax; // $TODO1 - Move into subsystem
import com.revrobotics.CANSparkMaxLowLevel.MotorType; // $TODO1 - Move into subsystem

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public final XboxController m_controller1;
  public final XboxController m_controller2;

  public final TankDriveSystem m_driveSystem;
  public final ArmSystem m_armSystem;
  public final GrabberSystem m_grabSystem;
  public final ActiveIntakeSystem m_intakeSystem;

  private final CANSparkMax m_activeIntakeMotor1; // $TODO1 - Move into subsystem
  private final CANSparkMax m_activeIntakeMotor2; // $TODO1 - Move into subsystem

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * 
   * <p>
   * Configure default command for Drive Subsystem. We want there to ALWAYS be a motor signal
   * sent, even in autonomous mode. If we didn't send a motor signal EVERY 20ms,
   * then Slew would prevent the robot from ever stopping fully in some cases.
   * Also, the motor watchdog would trip.
   * </p>
   */
  public RobotContainer() {
    // Create controllers
    m_controller1 = new XboxController(Constants.OperatorConstants.kDriverControllerPort1);
    m_controller2 = new XboxController(Constants.OperatorConstants.kDriverControllerPort2);

    // Crreate all subsystems
    m_driveSystem = TankDriveSystemSim.createTankDriveSystemInstance(m_controller1);
    m_armSystem = ArmSystemSim.createArmSystemInstance(m_controller2);
    m_grabSystem = GrabberSystemSim.createGrabberSystemInstance(m_controller2);

    // $TODO1 - Move into subsystem
    // Create intake motors
    m_activeIntakeMotor1 = new CANSparkMax(Constants.OperatorConstants.kActiveIntakeMotorChannel1,
        MotorType.kBrushless);
    m_activeIntakeMotor1.setInverted(true);
    // m_activeIntakeMotor1.burnFlash();

    if (Constants.OperatorConstants.kActiveIntakeMotorChannel2 != null) {
      m_activeIntakeMotor2 = new CANSparkMax(Constants.OperatorConstants.kActiveIntakeMotorChannel2,
          MotorType.kBrushless);
      // m_activeIntakeMotor2.burnFlash();
    }
    else {
      m_activeIntakeMotor2 = null;
    }

    m_intakeSystem = new ActiveIntakeSystem(m_activeIntakeMotor1, m_activeIntakeMotor2,
        m_controller2);

    setDefaultCommands();
    setupGrabberNotificationsToArm();
  }

  /**
   * Binds a button on the joystick to a command to run.
   */
  public void configureBindings() {
    new Trigger(m_controller2::getXButtonReleased).onTrue(
        new SetWinchToAngle(m_armSystem, Constants.OperatorConstants.kWinchMiddleNodeCube, 1));

    new Trigger(m_controller2::getYButtonReleased)
        .onTrue(new RetractArmCommand(m_armSystem).andThen(new SetSoftLimitCommand(m_armSystem)));
  }

  /**
   * When autonomous mode is started, this is the one command we run. It sequences
   * together lots of smaller commands.
   */
  public Command getAutonomousCommand() {
    System.out.println("Auto command scheduled container");
    return Auto.getAutoCommand(m_driveSystem, m_armSystem, m_grabSystem);
  }

  /**
   * This is the single point in code that sets up Shuffleboard. It calls
   * each subsystem in-turn to setup up Shuffleboard for that system.
   */
  public void initDashboard() {
    m_driveSystem.initDashBoard();
    m_armSystem.initDashBoard();
    m_grabSystem.initDashBoard();
  }

  /**
   * This is the single point in code that updates Shuffleboard.
   */
  public void updateDashBoard() {
    m_driveSystem.updateDashBoard();
    m_armSystem.updateDashBoard();
    m_grabSystem.updateDashBoard();
  }

  private void setDefaultCommands() {
    m_driveSystem.setDefaultCommand(m_driveSystem.getDefaultDriveCommand());
    m_armSystem.setDefaultCommand(m_armSystem.getDefaultArmCommand());
  }

  // Stitch together BooleanSupplier from GrabberSystemSim with ArmSystemSim
  private void setupGrabberNotificationsToArm() {
    if (m_grabSystem instanceof GrabberSystemSim && m_armSystem instanceof ArmSystemSim) {

      BooleanSupplier supplier = ((GrabberSystemSim) m_grabSystem).getGrabberOpenSupplier();
      ((ArmSystemSim) m_armSystem).setGrabberOpenSupplier(supplier);
    }
  }
}
