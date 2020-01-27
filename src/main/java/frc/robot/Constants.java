/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Varibles for Turning on Drivetrain
    public static double kTurnrateCurve = 0.1;
    public static double kTurnrateLimit = 0.8;
    
    //NAVX PID settings for  turning to angle
    public static double kToleranceDegrees = 5.0;
    public static double Turn_kP = 0.1;
    public static double Turn_kI = 0.0;
    public static double Turn_kD = 0.35;
    public static double Turn_kF = 0.0;
    public static double DesiredDistance;
    public static double DesiredHeading;
    //(4096 / (10 * (20/64) * (12/36) * (Constants.kDriveWheelDiameterInches*Math.PI))); // 200.0263
    // roughly 20k units/100ms at ~103 ips
    
    //Kinematics Varibales
    public static double kTrackLengthInches = 8.265;
    public static double kTrackWidthInches = 23.8;
    public static double kTrackEffectiveDiameter = (kTrackWidthInches * kTrackWidthInches
            + kTrackLengthInches * kTrackLengthInches) / kTrackWidthInches;
    public static double kTrackScrubFactor = 0.5;
    public static double kDriveWheelDiameterInches = 6.25;
    public static double kRatioFactor = 43;
    
    // PID gains for drive velocity loop
    // Units: error is 4096 counts/rev. Max output is +/- 1023 units.
    public static double kDriveVelocityKp = 0.1; //1.0
    public static double kDriveVelocityKi = 0.0;
    public static double kDriveVelocityKd = 1.0; //6.0
    public static double kDriveVelocityKf = 0.5;
    public static int kDriveVelocityIZone = 0;
    public static double kDriveVelocityRampRate = 0.0;
    public static int kDriveVelocityAllowableError = 0;
        
    // PID gains for drive base lock loop
    // Units: error is 4096 counts/rev. Max output is +/- 1023 units.
    public static double kDriveBaseLockKp = 0.5;
    public static double kDriveBaseLockKi = 0;
    public static double kDriveBaseLockKd = 0;
    public static double kDriveBaseLockKf = 0;
    public static int kDriveBaseLockIZone = 0;
    public static double kDriveBaseLockRampRate = 0;
    public static int kDriveBaseLockAllowableError = 10;
}
