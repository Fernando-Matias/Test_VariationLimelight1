 /*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.*;
import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.Utility.*;
import frc.robot.RobotMap;
import frc.robot.OI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static final DriveTrain instance = new DriveTrain();

  public static DriveTrain getInstance(){
    return instance;
  }
public static DefaultDriveTalonSRX mDrive_LeftMaster, mDrive_LeftB, mDrive_LeftC;
public static DefaultDriveTalonSRX mDrive_RightMaster, mDrive_RightB, mDrive_RightC;

public double TurnRateCurved;

public static DifferentialDrive mDrive;


public DriveTrain() {
  // Left Side
  mDrive_LeftMaster = new DefaultDriveTalonSRX(RobotMap.mDrive_Left_A_ID);
  mDrive_LeftB = new DefaultDriveTalonSRX(RobotMap.mDrive_Left_B_ID);
  mDrive_LeftC = new DefaultDriveTalonSRX(RobotMap.mDrive_Left_C_ID);

  // Right Side
  mDrive_RightMaster = new DefaultDriveTalonSRX(RobotMap.mDrive_Right_A_ID);
  mDrive_RightB = new DefaultDriveTalonSRX(RobotMap.mDrive_Right_B_ID);
  mDrive_RightC = new DefaultDriveTalonSRX(RobotMap.mDrive_Right_C_ID);

  mDrive_LeftB.set(ControlMode.Follower,RobotMap.mDrive_Left_A_ID);
  mDrive_LeftC.set(ControlMode.Follower,RobotMap.mDrive_Left_A_ID);

  mDrive_RightB.set(ControlMode.Follower,RobotMap.mDrive_Right_A_ID);
  mDrive_RightC.set(ControlMode.Follower,RobotMap.mDrive_Right_A_ID);


  mDrive = new DifferentialDrive(mDrive_LeftMaster, mDrive_RightMaster);
  mDrive.setSafetyEnabled(false);
}

public void setCoast() {
  mDrive_LeftMaster.setNeutralMode(NeutralMode.Coast);
  mDrive_LeftB.setNeutralMode(NeutralMode.Coast);
  mDrive_LeftC.setNeutralMode(NeutralMode.Coast);
  mDrive_RightMaster.setNeutralMode(NeutralMode.Coast);
  mDrive_RightB.setNeutralMode(NeutralMode.Coast);
  mDrive_RightC.setNeutralMode(NeutralMode.Coast);
}
public void setBrake() {
  mDrive_LeftMaster.setNeutralMode(NeutralMode.Brake);
  mDrive_LeftB.setNeutralMode(NeutralMode.Brake);
  mDrive_LeftC.setNeutralMode(NeutralMode.Brake);
  mDrive_RightMaster.setNeutralMode(NeutralMode.Brake);
  mDrive_RightB.setNeutralMode(NeutralMode.Brake);
  mDrive_RightC.setNeutralMode(NeutralMode.Brake);
}

public void EnableVoltComp() {
  mDrive_LeftMaster.enableVoltageCompensation(true);
  mDrive_LeftB.enableVoltageCompensation(true);
  mDrive_LeftC.enableVoltageCompensation(true);
  mDrive_RightMaster.enableVoltageCompensation(true);
  mDrive_RightB.enableVoltageCompensation(true);
  mDrive_RightC.enableVoltageCompensation(true);
}
public void DisableVoltComp() {
  mDrive_LeftMaster.enableVoltageCompensation(false);
  mDrive_LeftB.enableVoltageCompensation(false);
  mDrive_LeftC.enableVoltageCompensation(false);
  mDrive_RightMaster.enableVoltageCompensation(false);
  mDrive_RightB.enableVoltageCompensation(false);
  mDrive_RightC.enableVoltageCompensation(false);
}
public void StopDrivetrain() {
  mDrive_LeftMaster.set(ControlMode.PercentOutput, 0.0);
  mDrive_RightMaster.set(ControlMode.PercentOutput, 0.0);
}

public void Curvature(double ThrottleAxis, double TurnAxis) {
  TurnRateCurved = (Constants.kTurnrateCurve*Math.pow(TurnAxis,3)+(1-Constants.kTurnrateCurve)*TurnAxis*Constants.kTurnrateLimit);
  mDrive.curvatureDrive(ThrottleAxis, TurnRateCurved, true);
}

  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
