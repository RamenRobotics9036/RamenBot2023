package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.GrabberSystem;
import frc.robot.subsystems.TankDriveSystem;

public class Auto {
    private Auto() {
        throw new Error("Auto is a utility class and should not be constructed. One should utilize this class via static methods.");
    }

    public static CommandBase getAutoCommand(TankDriveSystem m_driveSystem, ArmSystem m_armSystem, GrabberSystem m_grabSystem) {
        return Commands.sequence(
            new DriveForwardCommand(m_driveSystem, 5, 5, 0.2), // Dummy constants
            new TurnInPlaceCommand(m_driveSystem, 5, 5, 0.2, true, false) // Dummy constants
            );
    }
} 