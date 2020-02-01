/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Setup;
import frc.robot.subsystems.*;

import edu.wpi.first.networktables.NetworkTableInstance;

public class TurrentShoot extends Command {

  private Timer shootTimer = new Timer();
  private boolean doneshooting = false;
  private Limelight limelight = Limelight.getInstance();

  double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

  private Setup setup = Setup.getInstance();

  public TurrentShoot() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setup.ShootCargo();
    shootTimer.start();

  }

  // Called repeatedly when this Command is scheduled to rung
  @Override
  protected void execute() {
    if (shootTimer.get() >= 1.0 && tx > 1.0 ) {

      double heading_error = tx;
      double steeringAdjust = 0.0f;

/*       if (tx> 1.0){
        steeringAdjust = limelight.kp*heading_error - limelight.minCommand;

      }
      else if (tx < 1.0){
        steeringAdjust = limelight.kp*heading_error + limelight.minCommand;
      }

      double leftCommand =+ steeringAdjust;
      double rightCommand =- steeringAdjust;
      DriveTrain.mDrive.tankDrive(leftCommand, rightCommand);
      //servoCam.set(steeringAdjust);
    } */

      shootTimer.stop();
      shootTimer.reset();
      setup.StopShootCargo();
      doneshooting = true;
    }
    else {
      doneshooting = false;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
