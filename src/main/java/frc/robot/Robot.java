/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.DriveTrain;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


public class Robot extends TimedRobot {

  DriveTrain driveTrain = new DriveTrain();
  HotSticks hotDrive = new HotSticks(0);
  HotSticks hotOp = new HotSticks(1);


  @Override
  public void robotInit() {
    
  }

  @Override
  public void autonomousInit() {
   
  }

  @Override
  public void autonomousPeriodic() {
  
  }

  @Override
  public void teleopPeriodic() {
    driveTrain.driveManualTank(1.0, 1.0, 1.0);

    // if(hotDrive.getButtonA() == true){
    //   driveTrain.dropH(true);
    // }else{
    //   driveTrain.dropH(false);
    // }
    // driveTrain.writeDashboard();
    //driveTrain.driveManualH(1, 1, 0.5, 1);
  }

  @Override
  public void testPeriodic() {
  }
}
