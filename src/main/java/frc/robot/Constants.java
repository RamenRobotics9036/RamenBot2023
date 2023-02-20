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
  public static class OperatorConstants {
    public static final int kDriverJoystickPort = 0;
    public static final int kDriverControllerPort = 1;

    public static final int kLeftMotorForwardChannel= 13;
    public static final int kLeftMotorBackChannel = 12;
    public static final int kRightMotorBackChannel = 10;
    public static final int kRightMotorForwardChannel = 11;

    public static final int kArmWinchChannel = 19;
    public static final int kArmExtenderChannel = 20;

    public static final int kGrabberForwardChannel = 0;
    public static final int kGrabberBackwardChannel = 1;

    public static boolean kSquareInputsDrive = true;
    public static boolean kSquareInputsArm = true;
    public static double kMaxOutput = 0.2;
    
    public static double kDeadband = 0.05;

    public static double kgearBoxRatio = 8.28;
    public static double kwheelDiameterMeter = 0.1524; // 6 inches
  }
}
