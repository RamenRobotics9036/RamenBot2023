package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.ArmSystem;
import frc.robot.Subsystems.GrabberSystem;
import frc.robot.Subsystems.TankDriveSystem;

public class Auto {
    public static final String kAutoModeKey = "Auto Mode";
    public static final String kDropAndDriveMode = "Score and Move";
    public static final String kAutoBalanceMode = "Auto Balance";
    public static final String kSimpleMode = "Only Score";
    public static final String kDefaultAutoModeValue = kSimpleMode;

    private Auto() {
        throw new Error("Auto is a utility class and should not be constructed. One should utilize this class via static methods.");
    }

    private static String getSelectedAutoMode() {
      Sendable retrievedChooserVal = SmartDashboard.getData(Auto.kAutoModeKey);

      if (retrievedChooserVal == null) {
        System.out.println("UNEXPECTED: Got back null for smartdash chooser");
        return "";
      }
    
      SendableChooser<String> retrievedChooser = (SendableChooser<String>)retrievedChooserVal;
      return retrievedChooser.getSelected();
    }

    public static CommandBase getAutoCommand(TankDriveSystem m_driveSystem, ArmSystem m_armSystem, GrabberSystem m_grabSystem) {
      String autoMode = getSelectedAutoMode();
      System.out.println("Auto mode selected: " + autoMode);

      switch (autoMode) {

        case kDropAndDriveMode:
            return Commands.sequence(
              new SetWinchToAngle(m_armSystem, 0.75, 0.9),
              new SetExtenderToLength(m_armSystem, -100, 0.9),
              new WaitCommand(0.5),
              new GrabberToggleCommand(m_grabSystem),
              new WaitCommand(0.5),
              new DriveCommand(m_driveSystem, 15 * 12, Constants.OperatorConstants.kGearBoxRatioDrive, 0.5, Constants.OperatorConstants.kWheelCircumferenceInchesDrive)      
            );

          case kAutoBalanceMode:
            return Commands.sequence(
              new SetWinchToAngle(m_armSystem, 0.75, 0.9),
              new SetExtenderToLength(m_armSystem, -100, 0.9),
              new WaitCommand(0.5),
              new GrabberToggleCommand(m_grabSystem),
              new WaitCommand(0.5),
              new DriveCommand(m_driveSystem, 0.5 * 12, Constants.OperatorConstants.kGearBoxRatioDrive, -0.4, Constants.OperatorConstants.kWheelCircumferenceInchesDrive),
              new TurnDegrees(m_driveSystem, 0.5, 75),
              new WaitCommand(0.5),
              new DriveCommand(m_driveSystem, 8 * 12, Constants.OperatorConstants.kGearBoxRatioDrive, -0.4, Constants.OperatorConstants.kWheelCircumferenceInchesDrive),
              new AutoBalanceCommand(m_driveSystem, 0.25) 
          );

        case kSimpleMode:
          return Commands.sequence(
              new SetWinchToAngle(m_armSystem, 0.75, 0.9),
              new SetExtenderToLength(m_armSystem, -100, 0.9),
              new WaitCommand(0.5),
              new GrabberToggleCommand(m_grabSystem)
         );

        default:
            System.out.println("UNEXPECTED AUTO MODE - auto mode will do nothing");
            return new InstantCommand();        
      }

    }
                
    public static void putShuffleBoardCommands(TankDriveSystem m_driveSystem, ArmSystem m_armSystem, GrabberSystem m_grabSystem) {

        // SmartDashboard.putBoolean("Auto Middle", false);
        // SmartDashboard.putData("Retract Arm", new RetractArmCommand(m_armSystem));
        // SmartDashboard.putData("Set Soft Limit", new SetSoftLimitCommand(m_armSystem));
    //     SmartDashboard.putData("Rotate Winch Forwards", m_armSystem.rotateWinchMotor(
    //         SmartDashboard.getNumber("Auto Motor Distance", Constants.OperatorConstants.kAutoMotorDistance),
    //          Constants.OperatorConstants.kGearBoxRatioArm,
    //          SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed),
    //          Constants.OperatorConstants.kWheelDiameterInchWinch));

    //     SmartDashboard.putData("Rotate Winch Backwards", m_armSystem.rotateWinchMotor(
    //     SmartDashboard.getNumber("Auto Motor Distance", Constants.OperatorConstants.kAutoMotorDistance),
    //         Constants.OperatorConstants.kGearBoxRatioArm,
    //         -SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed),
    //         Constants.OperatorConstants.kWheelDiameterInchWinch));

    //     SmartDashboard.putData("Rotate Extender Forwards", m_armSystem.rotateExtenderMotor(
    //         SmartDashboard.getNumber("Auto Motor Distance", Constants.OperatorConstants.kAutoMotorDistance),
    //             Constants.OperatorConstants.kGearBoxRatioArm,
    //             SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed),
    //             Constants.OperatorConstants.kWheelDiameterInchExtender));

    //     SmartDashboard.putData("Rotate Extender Backwards", m_armSystem.rotateExtenderMotor(
    //     SmartDashboard.getNumber("Auto Motor Distance", Constants.OperatorConstants.kAutoMotorDistance),
    //         Constants.OperatorConstants.kGearBoxRatioArm,
    //         -SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed),
    //         Constants.OperatorConstants.kWheelDiameterInchExtender));

    //     SmartDashboard.putData("Turn In Place", new TurnInPlaceCommand(m_driveSystem,
    //      SmartDashboard.getNumber("Auto Motor Distance", Constants.OperatorConstants.kAutoMotorDistance),
    //       Constants.OperatorConstants.kGearBoxRatioDrive, SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed),
    //     SmartDashboard.getBoolean("Auto Turn Left", true),
    //     Constants.OperatorConstants.kWheelCircumferenceInchesDrive));

    //     SmartDashboard.putData("Drive Forwards", new DriveCommand(m_driveSystem,
    //      SmartDashboard.getNumber("Auto Motor Distance",
    //       Constants.OperatorConstants.kAutoMotorDistance),
    //        Constants.OperatorConstants.kGearBoxRatioDrive,
    //        SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed),
    //        Constants.OperatorConstants.kWheelCircumferenceInchesDrive));

    //     SmartDashboard.putData("Drive Backwards", new DriveCommand(m_driveSystem,
    //      SmartDashboard.getNumber("Auto Motor Distance",
    //       Constants.OperatorConstants.kAutoMotorDistance),
    //        Constants.OperatorConstants.kGearBoxRatioDrive,
    //        -SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed),
    //        Constants.OperatorConstants.kWheelCircumferenceInchesDrive));

    //     SmartDashboard.putData("Grab Cargo", new GrabberToggleCommand(m_grabSystem));
    }
}