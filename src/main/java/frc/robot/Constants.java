// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /**
   * Consants for human players and controllers on a laptop.
   */
  public static class ControllerConstants {
    public static int driveControllerPort = 0;
    public static int armControllerPort = 1;
  }

  /**
   * Consants for systems on the robot.
   */
  public static class RobotConstants {
    public static int frontRightMotorChannel = 10;
    public static int backRightMotorChannel = 11;
    public static int frontLeftMotorChannel = 13;
    public static int backLeftMotorChannel = 12;

    public static int solenoidForwardChannel = 0;
    public static int solenoidBackChannel = 15;

    public static int winchMotorChannel = 19;
    public static int extenderMotorChannel = 20;
  }

  /**
   * Consants for system paramaters.
   */
  public static class SystemConstants {
    public static boolean squareInputsDrive = true;
    public static double maxOutputDrive = 0.5;
    public static double deadBandDrive= 0.1;

    public static boolean squareInputsArm = true;
    public static double deadBandArm= 0.1;
    public static double winchMaxOutput = 0.5;
    public static double extenderMaxOutput = 0.5;
  }

  /**
   * Consants for autonomous parameters.
   */
  public static class AutoConstants {
    public static double gearBoxRatioDrive = 8.28;
    public static double wheelDiameterInchDrive = 6;
    public static boolean autoUseChargeStation = false;
  }
}
