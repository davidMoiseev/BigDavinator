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

  int state;

  @Override
  public void robotInit() {
    
  }

  @Override
  public void autonomousInit() {
   driveTrain.zeroEncoders();
  driveTrain.DrivetrainConfig();
  }

  @Override
  public void autonomousPeriodic() {
    //driveTrain.driveStraightMagically(670.0, 6000, 6000, 2.0, 0.03, 0.000008, 0.0);

  switch(state){
    case 0:
      if(driveTrain.driveStraightMagically(100000.0, 3000, 21000, 1, 0.0, 0.0, 0.0) == true){
        state++;
      }
    }

    //driveTrain.driveStraightMagically(670.0, 6000, 6000, feed forward, proportional, integral, derivative); 
    //0.345813685
    driveTrain.writeDashboard();

  }

  @Override
  public void teleopInit(){
    driveTrain.DrivetrainConfig();
  }

  @Override
  public void teleopPeriodic() {
    driveTrain.driveManualH(1.0, 1.0, 1.0, 1.0);


    driveTrain.writeDashboard();
  }

  @Override
  public void testPeriodic() {
  }
}
