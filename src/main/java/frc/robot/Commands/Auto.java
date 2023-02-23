package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.ArmSystem;
import frc.robot.Subsystems.GrabberSystem;
import frc.robot.Subsystems.TankDriveSystem;

public class Auto {
    private Auto() {
        throw new Error("Auto is a utility class and should not be constructed. One should utilize this class via static methods.");
    }

    public static CommandBase getAutoCommand(TankDriveSystem m_driveSystem, ArmSystem m_armSystem, GrabberSystem m_grabSystem) {
        return Commands.sequence(
            new TurnInPlaceCommand(m_driveSystem, 5, 8, 0.2, false)
            ); // All constants used are dummy constants
    }

    public static void initShuffleBoardCommands(TankDriveSystem m_driveSystem, ArmSystem m_armSystem, GrabberSystem m_grabSystem) {
        SmartDashboard.putData("Rotate Winch Forwards", m_armSystem.rotateWinchMotor(1, 60, 0.35));
        SmartDashboard.putData("Rotate Winch Backwards", m_armSystem.rotateWinchMotor(1, 60, -0.35));
        SmartDashboard.putData("Rotate Extender Backwards", m_armSystem.rotateExtenderMotor(1, 60, -0.35));
        SmartDashboard.putData("Rotate Extender Backwards", m_armSystem.rotateExtenderMotor(1, 60, 0.35));
        SmartDashboard.putData("Turn In Place", new TurnInPlaceCommand(m_driveSystem, 1, 8, 0.35, true));
        SmartDashboard.putData("Drive Forwards", new DriveForwardCommand(m_driveSystem, 1, 8, 0.35));
        SmartDashboard.putData("Drive Backwards", new DriveForwardCommand(m_driveSystem, 1, 8, 0.35));
        SmartDashboard.putData("Grab Cargo", new GrabberToggleCommand(m_grabSystem));
    }
}