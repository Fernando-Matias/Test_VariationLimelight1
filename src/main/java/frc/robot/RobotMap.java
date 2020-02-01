/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
  public static final int mDrive_Left_A_ID = 3;
  public static final int mDrive_Left_B_ID = 5;
  public static final int mDrive_Left_C_ID = 7;
  public static final int mDrive_Right_A_ID = 4;
  public static final int mDrive_Right_B_ID = 6;
  public static final int mDrive_Right_C_ID = 8;

  public static final int TurrentShooter_ID= 10;

  public static final int LeftStickPort = 0;
  public static final int RightStickPort = 1;
  public static final int GamepadPort = 2;

}
