// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.RetractArmCommand;
import frc.robot.Commands.Auto;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private AddressableLED m_LEDLight = new AddressableLED(
      Constants.OperatorConstants.kLEDLightsChannel);
  private AddressableLEDBuffer m_LEDBuffer = new AddressableLEDBuffer(
      Constants.OperatorConstants.kLEDLightsLength);
  private VerifyJoysticks m_verifyJoysticks;
  private SendableChooser<String> m_chooser;
  private int m_ledLoop;
  private int m_ledR;
  private int m_ledG;
  private int m_ledB;
  private int m_ledHue;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_verifyJoysticks = new VerifyJoysticks(VerifyJoysticks.GetDefaultJoystickConfigs(),
        new DriverStationFunctions(), 1);

    m_robotContainer = new RobotContainer();
    m_robotContainer.initDashboard();
    SmartDashboard.putBoolean("Get Cube", true);
    m_chooser = Auto.addAutoModeChooser();
    m_ledLoop = 0;
    m_ledR = 0;
    m_ledG = 0;
    m_ledB = 0;
    m_ledHue = 0;
    m_LEDLight.setLength(m_LEDBuffer.getLength());
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard
   * integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    m_verifyJoysticks.VerifyJoysticksPeriodically();
    updateLeds();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_ledR = 0;
    m_ledG = 0;
    m_ledB = 0;

    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.putShuffleBoardAutoCommands();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_autonomousCommand.schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    m_ledR = 0;
    m_ledG = 0;
    m_ledB = 0;

    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.configureBindings();

    m_robotContainer.putShuffleBoardAutoCommands();
    m_robotContainer.updateDashBoard();

    new RetractArmCommand(m_robotContainer.m_armSystem).schedule();

    // m_LEDLight.setLength(m_LEDBuffer.getLength());
    SmartDashboard.putNumber("Winch Encoder",
        m_robotContainer.m_armSystem.getWinchAbsoluteEncoder());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    if (m_robotContainer.m_controller2.getLeftTriggerAxis() > 0.05) {
      SmartDashboard.putBoolean("Get Cube", true);
      m_ledR = 255;
      m_ledG = 255;
      m_ledB = 0;

      // for (var i = 0; i < m_LEDBuffer.getLength(); i++) {
      // if(i==m_loop) {
      // m_LEDBuffer.setRGB(i, 0, 0, 0);
      // } else {
      // m_LEDBuffer.setRGB(i, 0, 255, 255);
      // }
      // }
      // m_LEDLight.setData(m_LEDBuffer);
      // m_LEDLight.start();
    }
    else if (m_robotContainer.m_controller2.getRightTriggerAxis() > 0.05) {
      SmartDashboard.putBoolean("Get Cube", false);

      m_ledR = 255;
      m_ledG = 0;
      m_ledB = 255;

      // for (var i = 0; i < m_LEDBuffer.getLength(); i++) {
      // if(i==m_loop) {
      // m_LEDBuffer.setRGB(i, 0, 0, 0);
      // } else {
      // m_LEDBuffer.setRGB(i, 150, 255, 0);
      // }
      // }
      // m_LEDLight.setData(m_LEDBuffer);
      // m_LEDLight.start();
    }
    // updateLeds();
  }

  private void updateLeds() {
    if (0 == m_ledR && 0 == m_ledG && 0 == m_ledB) {
      for (var i = 0; i < m_LEDBuffer.getLength() / 2; i++) {
        var hue = (m_ledHue + (i * 180 / (m_LEDBuffer.getLength() / 2))) % 180;
        m_LEDBuffer.setHSV(i, hue, 255, 128);
        m_LEDBuffer.setHSV(m_LEDBuffer.getLength() - i - 1, hue, 255, 128);
      }
      m_ledHue += 2;
      m_ledHue %= 180;

    }
    else {
      for (var i = 0; i < m_LEDBuffer.getLength() / 2; i++) {
        if (i == m_ledLoop) {
          m_LEDBuffer.setRGB(i, 0, 0, 0);
          m_LEDBuffer.setRGB(m_LEDBuffer.getLength() - i - 1, 0, 0, 0);
        }
        else {
          m_LEDBuffer.setRGB(i, m_ledG, m_ledR, m_ledB);
          m_LEDBuffer.setRGB(m_LEDBuffer.getLength() - i - 1, m_ledG, m_ledR, m_ledB);
        }
      }
    }
    m_LEDLight.setData(m_LEDBuffer);
    m_LEDLight.start();

    m_ledLoop -= 1;
    if (m_ledLoop < 0) {
      m_ledLoop = m_LEDBuffer.getLength() / 2;
    }
  }

  @Override

  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  @Override
  public void disabledInit() {
    m_ledR = 0;
    m_ledG = 0;
    m_ledB = 0;
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.m_driveSystem.calibrate();
  }
}
