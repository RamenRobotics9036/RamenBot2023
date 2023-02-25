package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Subsystems.ArmSystem;
import frc.robot.Subsystems.GrabberSystem;
import frc.robot.Subsystems.TankDriveSystem;

public class Auto {
    private Auto() {
        throw new Error("Auto is a utility class and should not be constructed. One should utilize this class via static methods.");
    }

    public static CommandBase getAutoCommand(TankDriveSystem m_driveSystem, ArmSystem m_armSystem, GrabberSystem m_grabSystem) {
        if (SmartDashboard.getNumber("Starting Position", 0) == 0) {
            return Commands.sequence(
                new DriveCommand(m_driveSystem, 1, Constants.OperatorConstants.kGearBoxRatioDrive, 0.3, Constants.OperatorConstants.kWheelCircumferenceInchesDrive),
                // Score Piece Here
                new DriveCommand(m_driveSystem, 15 * 12, Constants.OperatorConstants.kGearBoxRatioDrive, -0.6, Constants.OperatorConstants.kWheelCircumferenceInchesDrive)
                // Put Down Winch And Extender

            );
        } else if (SmartDashboard.getNumber("Starting Position", 0) == 1) {
            return Commands.sequence(
                new DriveCommand(m_driveSystem, 1, Constants.OperatorConstants.kGearBoxRatioDrive, 0.3, Constants.OperatorConstants.kWheelCircumferenceInchesDrive)
                // Score Piece Here
                // Put Down Winch And Extender
            );
        } else if (SmartDashboard.getNumber("Starting Position", 0) == 2) {
            return Commands.sequence(
                new DriveCommand(m_driveSystem, 1, Constants.OperatorConstants.kGearBoxRatioDrive, 0.3, Constants.OperatorConstants.kWheelCircumferenceInchesDrive),
                // Score Piece Here
                new DriveCommand(m_driveSystem, 15 * 12, Constants.OperatorConstants.kGearBoxRatioDrive, -0.6, Constants.OperatorConstants.kWheelCircumferenceInchesDrive)
                // Put Down Winch And Extender
            );
        }
        return null; // 0 is left, 1 is middle, 2 is right    
    }
    public static void putShuffleBoardCommands(TankDriveSystem m_driveSystem, ArmSystem m_armSystem, GrabberSystem m_grabSystem) {
        SmartDashboard.putData("Rotate Winch Forwards", m_armSystem.rotateWinchMotor(
            SmartDashboard.getNumber("Auto Motor Distance", Constants.OperatorConstants.kAutoMotorDistance),
             Constants.OperatorConstants.kGearBoxRatioArm,
             SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed),
             Constants.OperatorConstants.kWheelDiameterInchWinch));

        SmartDashboard.putData("Rotate Winch Backwards", m_armSystem.rotateWinchMotor(
        SmartDashboard.getNumber("Auto Motor Distance", Constants.OperatorConstants.kAutoMotorDistance),
            Constants.OperatorConstants.kGearBoxRatioArm,
            -SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed),
            Constants.OperatorConstants.kWheelDiameterInchWinch));

        SmartDashboard.putData("Rotate Extender Forwards", m_armSystem.rotateExtenderMotor(
            SmartDashboard.getNumber("Auto Motor Distance", Constants.OperatorConstants.kAutoMotorDistance),
                Constants.OperatorConstants.kGearBoxRatioArm,
                SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed),
                Constants.OperatorConstants.kWheelDiameterInchExtender));

        SmartDashboard.putData("Rotate Extender Backwards", m_armSystem.rotateExtenderMotor(
        SmartDashboard.getNumber("Auto Motor Distance", Constants.OperatorConstants.kAutoMotorDistance),
            Constants.OperatorConstants.kGearBoxRatioArm,
            -SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed),
            Constants.OperatorConstants.kWheelDiameterInchExtender));

        SmartDashboard.putData("Turn In Place", new TurnInPlaceCommand(m_driveSystem,
         SmartDashboard.getNumber("Auto Motor Distance", Constants.OperatorConstants.kAutoMotorDistance),
          Constants.OperatorConstants.kGearBoxRatioDrive, SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed),
        SmartDashboard.getBoolean("Auto Turn Left", true),
        Constants.OperatorConstants.kWheelCircumferenceInchesDrive));

        SmartDashboard.putData("Drive Forwards", new DriveCommand(m_driveSystem,
         SmartDashboard.getNumber("Auto Motor Distance",
          Constants.OperatorConstants.kAutoMotorDistance),
           Constants.OperatorConstants.kGearBoxRatioDrive,
           SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed),
           Constants.OperatorConstants.kWheelCircumferenceInchesDrive));

        SmartDashboard.putData("Drive Backwards", new DriveCommand(m_driveSystem,
         SmartDashboard.getNumber("Auto Motor Distance",
          Constants.OperatorConstants.kAutoMotorDistance),
           Constants.OperatorConstants.kGearBoxRatioDrive,
           -SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed),
           Constants.OperatorConstants.kWheelCircumferenceInchesDrive));

        SmartDashboard.putData("Grab Cargo", new GrabberToggleCommand(m_grabSystem));
    }
}