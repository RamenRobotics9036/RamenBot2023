// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.GrabSystem;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase getAuto(DriveSystem m_driveSystem, ArmSystem m_armSystem, GrabSystem m_grabSystem) {
    if (SmartDashboard.getBoolean("Auto Charge Station", AutoConstants.autoUseChargeStation)) {
      System.out.println("USING CHARGE STATION IN AUTO");
      return Commands.sequence(
        new InstantCommand()
      );
    } else {
      System.out.println("USING STANDARD AUTO COMMAND");
      return Commands.sequence(
        new InstantCommand()
      );
    }
  }

  public static void dashBoardInit() {
    SmartDashboard.putBoolean("Use Auto Charge Station", AutoConstants.autoUseChargeStation);
    System.out.println("USE AUTO CHARGE STATION SET TO " + SmartDashboard.getBoolean("Use Auto Charge Station", AutoConstants.autoUseChargeStation));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
