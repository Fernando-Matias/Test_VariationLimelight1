/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.*;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;



import frc.robot.RobotMap;


/**
 * Add your docs here.
 */
public class Limelight extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static final Limelight instance = new Limelight();

  public static Limelight getInstance(){
    return instance;
  }
  
  public double kp = -0.1f;
  public double minCommand = 0.05f;

  public boolean m_LimelightHasValidTarget = false;
  public double m_LimelightDriveCommand = 4.6;
  public double m_LimelightSteerCommand = 2.0;



  double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  

  /* public Limelight() {

    TurrentShooter = new TalonSRX(RobotMap.TurrentShooter_ID);


    TurrentShooter.configFactoryDefault();
    TurrentShooter.setNeutralMode(NeutralMode.Brake);
    TurrentShooter.configContinuousCurrentLimit(30);
    TurrentShooter.configPeakCurrentLimit(0);
    TurrentShooter.enableCurrentLimit(true);
    TurrentShooter.setInverted(false);    

 
    }

  } */

  public void UpdateLimelightTracking() {
    // These numbers must be tuned for your Robot! Be careful!
    final double STEER_K = 0.03; // how hard to turn toward the target
    final double DRIVE_K = 0.45; // how hard to drive fwd toward the target
    final double DESIRED_TARGET_AREA = 2.0; // Area of the target when the robot reaches the wall
    final double MAX_DRIVE = 0.7; // Simple speed limit so we don't drive too fast

    if (tv == 1.0) {
      m_LimelightHasValidTarget = true;
      // m_LimelightDriveCommand = 0.0;
      // m_LimelightSteerCommand = 0.0;
      return;
    } else {
      m_LimelightHasValidTarget = false;
    }

    // Start with proportional steering
    double steer_cmd = tx * STEER_K;
    m_LimelightSteerCommand = steer_cmd;

    // try to drive forward until the target area reaches our desired area
    double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

    // don't let the robot drive too fast into the goal
    if (drive_cmd > MAX_DRIVE) {
      drive_cmd = MAX_DRIVE;
    }
    m_LimelightDriveCommand = drive_cmd;
  }

 /*  public void ShootPowerCell() {
    TurrentShooter.set(ControlMode.PercentOutput, 1.0);
  }
  public void StopShootPowerCell() {
    TurrentShooter.set(ControlMode.PercentOutput, 0.0);
  } */

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    /* NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area); */
  }
}
