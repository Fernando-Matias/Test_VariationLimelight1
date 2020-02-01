/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


import frc.robot.RobotMap;

// import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Setup extends Subsystem {

    
    private static final Setup instance = new Setup();
    
    public static Setup getInstance() {
      return instance;
    }

  
    public TalonSRX TurrentShooter;

    public Setup(){

        TurrentShooter = new TalonSRX(RobotMap.TurrentShooter_ID);

        TurrentShooter.configFactoryDefault();
        TurrentShooter.setNeutralMode(NeutralMode.Brake);
        TurrentShooter.configContinuousCurrentLimit(30);
        TurrentShooter.configPeakCurrentLimit(0);
        TurrentShooter.enableCurrentLimit(true);
        TurrentShooter.setInverted(false);

/*         @Override
        protected void initialize() {
        Limelight.ShootPowerCell();
        shootTimer.start();
        }
    
        @Override
        protected void execute() {
        if (shootTimer.get() >= 1.0) {
          shootTimer.stop();
          shootTimer.reset();
          tray.StopShootCargo();
          doneshooting = true;
        } 
        else {
          doneshooting = false; */
    }
        
    public void ShootCargo() {
        TurrentShooter.set(ControlMode.PercentOutput, 1.0);
    }
    public void StopShootCargo() {
        TurrentShooter.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    protected void initDefaultCommand() {
        // TODO Auto-generated method stub

    }
    
}
