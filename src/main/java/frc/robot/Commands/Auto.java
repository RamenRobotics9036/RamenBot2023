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
        return getCommandSequence(m_driveSystem, m_armSystem, m_grabSystem);
    }

    private static CommandBase getCommandSequence(TankDriveSystem m_driveSystem, ArmSystem m_armSystem, GrabberSystem m_grabSystem) {
        if (SmartDashboard.getNumber("Position", 0) == 0) {
            return Commands.sequence(
                new DriveForwardCommand(m_driveSystem, 1, Constants.OperatorConstants.kGearBoxRatioDrive, 0.5)
            );
        } else if (SmartDashboard.getNumber("Position", 0) == 1) {
            return Commands.sequence(null);
        } else if (SmartDashboard.getNumber("Position", 0) == 2) {
            return Commands.sequence(null);
        }
        return null; // 0 is left, 1 is middle, 2 is right
    }

    public static void initShuffleBoardCommands(TankDriveSystem m_driveSystem, ArmSystem m_armSystem, GrabberSystem m_grabSystem) {
        SmartDashboard.putNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed);
        SmartDashboard.putNumber("Auto Motor Rotations", Constants.OperatorConstants.kAutoMotorRotations);
        SmartDashboard.putBoolean("Auto Turn Left", true);
        SmartDashboard.putNumber("Starting Position", 0);

        SmartDashboard.putData("Rotate Winch Forwards", m_armSystem.rotateWinchMotor(
            SmartDashboard.getNumber("Auto Motor Rotations", Constants.OperatorConstants.kAutoMotorRotations),
             Constants.OperatorConstants.kGearBoxRatioArm,
             SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed)));

        SmartDashboard.putData("Rotate Winch Backwards", m_armSystem.rotateWinchMotor(
        SmartDashboard.getNumber("Auto Motor Rotations", Constants.OperatorConstants.kAutoMotorRotations),
            Constants.OperatorConstants.kGearBoxRatioArm,
            -SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed)));

        SmartDashboard.putData("Rotate Extender Forwards", m_armSystem.rotateExtenderMotor(
            SmartDashboard.getNumber("Auto Motor Rotations", Constants.OperatorConstants.kAutoMotorRotations),
                Constants.OperatorConstants.kGearBoxRatioArm,
                SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed)));

        SmartDashboard.putData("Rotate Extender Backwards", m_armSystem.rotateExtenderMotor(
        SmartDashboard.getNumber("Auto Motor Rotations", Constants.OperatorConstants.kAutoMotorRotations),
            Constants.OperatorConstants.kGearBoxRatioArm,
            -SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed)));

        SmartDashboard.putData("Turn In Place", new TurnInPlaceCommand(m_driveSystem,
         SmartDashboard.getNumber("Auto Motor Rotations", Constants.OperatorConstants.kAutoMotorRotations),
          Constants.OperatorConstants.kGearBoxRatioDrive, SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed),
        SmartDashboard.getBoolean("Auto Turn Left", true)));

        SmartDashboard.putData("Drive Forwards", new DriveForwardCommand(m_driveSystem,
         SmartDashboard.getNumber("Auto Motor Rotations",
          Constants.OperatorConstants.kAutoMotorRotations),
           Constants.OperatorConstants.kGearBoxRatioDrive,
           SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed)));

        SmartDashboard.putData("Drive Backwards", new DriveForwardCommand(m_driveSystem,
         SmartDashboard.getNumber("Auto Motor Rotations",
          Constants.OperatorConstants.kAutoMotorRotations),
           Constants.OperatorConstants.kGearBoxRatioDrive,
           -SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed)));

        SmartDashboard.putData("Grab Cargo", new GrabberToggleCommand(m_grabSystem));
    }

    public static void putShuffleboardCommands(TankDriveSystem m_driveSystem, ArmSystem m_armSystem, GrabberSystem m_grabSystem) {
        SmartDashboard.putData("Rotate Winch Forwards", m_armSystem.rotateWinchMotor(
            SmartDashboard.getNumber("Auto Motor Rotations", Constants.OperatorConstants.kAutoMotorRotations),
             Constants.OperatorConstants.kGearBoxRatioArm,
             SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed)));

        SmartDashboard.putData("Rotate Winch Backwards", m_armSystem.rotateWinchMotor(
        SmartDashboard.getNumber("Auto Motor Rotations", Constants.OperatorConstants.kAutoMotorRotations),
            Constants.OperatorConstants.kGearBoxRatioArm,
            -SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed)));

        SmartDashboard.putData("Rotate Extender Forwards", m_armSystem.rotateExtenderMotor(
            SmartDashboard.getNumber("Auto Motor Rotations", Constants.OperatorConstants.kAutoMotorRotations),
                Constants.OperatorConstants.kGearBoxRatioArm,
                SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed)));

        SmartDashboard.putData("Rotate Extender Backwards", m_armSystem.rotateExtenderMotor(
        SmartDashboard.getNumber("Auto Motor Rotations", Constants.OperatorConstants.kAutoMotorRotations),
            Constants.OperatorConstants.kGearBoxRatioArm,
            -SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed)));

        SmartDashboard.putData("Turn In Place", new TurnInPlaceCommand(m_driveSystem,
         SmartDashboard.getNumber("Auto Motor Rotations", Constants.OperatorConstants.kAutoMotorRotations),
          Constants.OperatorConstants.kGearBoxRatioDrive, SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed),
        SmartDashboard.getBoolean("Auto Turn Left", true)));

        SmartDashboard.putData("Drive Forwards", new DriveForwardCommand(m_driveSystem,
         SmartDashboard.getNumber("Auto Motor Rotations",
          Constants.OperatorConstants.kAutoMotorRotations),
           Constants.OperatorConstants.kGearBoxRatioDrive,
           SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed)));

        SmartDashboard.putData("Drive Backwards", new DriveForwardCommand(m_driveSystem,
         SmartDashboard.getNumber("Auto Motor Rotations",
          Constants.OperatorConstants.kAutoMotorRotations),
           Constants.OperatorConstants.kGearBoxRatioDrive,
           -SmartDashboard.getNumber("Auto Motor Speed", Constants.OperatorConstants.kAutoMotorSpeed)));

        SmartDashboard.putData("Grab Cargo", new GrabberToggleCommand(m_grabSystem));
    }
}