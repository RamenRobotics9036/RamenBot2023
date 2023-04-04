package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.ArmSystem;
import frc.robot.Subsystems.GrabberSystem;
import frc.robot.Subsystems.TankDriveSystem;

public class Auto {
    private Auto() {
        throw new Error("Auto is a utility class and should not be constructed. One should utilize this class via static methods.");
    }

    public static CommandBase getAutoCommand(TankDriveSystem m_driveSystem, ArmSystem m_armSystem, GrabberSystem m_grabSystem) {              
                if (!SmartDashboard.getBoolean("Use AutoBalance", false)) {
                    return Commands.sequence(
                        new SetWinchToAngle(m_armSystem, 0.75, 0.9),
                        new SetExtenderToLength(m_armSystem, -100, 0.9),
                        new WaitCommand(0.5),
                        new GrabberToggleCommand(m_grabSystem),
                        new WaitCommand(0.5),
                        new DriveCommand(m_driveSystem, 15 * 12, Constants.OperatorConstants.kGearBoxRatioDrive, 0.5, Constants.OperatorConstants.kWheelCircumferenceInchesDrive)    
                    );
                }

                    // new SetWinchToAngle(m_armSystem, 0.75, 0.9),
                    // new SetExtenderToLength(m_armSystem, -100, 0.9),
                    // new WaitCommand(0.5),
                    // new GrabberToggleCommand(m_grabSystem),
                    // new TurnDegrees(m_driveSystem, 0.5, 80),
                    // new WaitCommand(0.5),
                    // new DriveCommand(m_driveSystem, 8 * 12, Constants.OperatorConstants.kGearBoxRatioDrive, 0.4, Constants.OperatorConstants.kWheelCircumferenceInchesDrive),
                    // new AutoBalanceCommand(m_driveSystem, 0.25)

                    return Commands.sequence(
                        new SetWinchToAngle(m_armSystem, 0.75, 0.9),
                        new SetExtenderToLength(m_armSystem, -100, 0.9),
                        new WaitCommand(0.25),
                        new GrabberToggleCommand(m_grabSystem),
                        new TurnDegrees(m_driveSystem, 0.5, 80),
                        new WaitCommand(0.25),
                        new DriveCommand(m_driveSystem, 8 * 12, Constants.OperatorConstants.kGearBoxRatioDrive, 0.4, Constants.OperatorConstants.kWheelCircumferenceInchesDrive),
                        new AutoBalanceCommand(m_driveSystem, 0.25)
                    );
                }
                
                // Not Autobalance
                // new SetWinchToAngle(m_armSystem, 0.75, 0.9),
                // new SetExtenderToLength(m_armSystem, -100, 0.9),
                // new WaitCommand(0.5),
                // new GrabberToggleCommand(m_grabSystem),
                // new WaitCommand(0.5),
                // new DriveCommand(m_driveSystem, 15 * 12, Constants.OperatorConstants.kGearBoxRatioDrive, 0.5, Constants.OperatorConstants.kWheelCircumferenceInchesDrive)

                // Autobalance
                // new SetWinchToAngle(m_armSystem, 0.75, 0.9),
                // new SetExtenderToLength(m_armSystem, -100, 0.9),
                // new WaitCommand(0.5),
                // new GrabberToggleCommand(m_grabSystem),
                // new TurnDegrees(m_driveSystem, 0.5, 80)
                // new WaitCommand(0.5), 
                // new DriveCommand(m_driveSystem, 8 * 12, Constants.OperatorConstants.kGearBoxRatioDrive, 0.4, Constants.OperatorConstants.kWheelCircumferenceInchesDrive),
                // new AutoBalanceCommand(m_driveSystem, 0.25)

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