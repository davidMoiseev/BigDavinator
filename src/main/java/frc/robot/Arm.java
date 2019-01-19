/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * Add your docs here.
 */
public class Arm {

  public static final int TALON_LEFT = 1;

  WPI_TalonSRX Arm = new WPI_TalonSRX(TALON_LEFT);
  Joystick joystick = new Joystick(0);
  StringBuilder stringbuilder = new StringBuilder();

  void setArmPos(double Arm, double Wrist){



  }


  private void RunArm(){

  
   

  } 
   
  void ArmConfig(){
    
    Arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,2, 100);
    Arm.setSensorPhase(true);
		Arm.setInverted(false);
    Arm.selectProfileSlot(1, 2);
    Arm.configNominalOutputForward(0, 100);
		Arm.configNominalOutputReverse(0, 100);
		Arm.configPeakOutputForward(1, 100);
		Arm.configPeakOutputReverse(-1, 100);

    Arm.configMotionCruiseVelocity(15000, 100);
    Arm.configMotionAcceleration(6000, 100);
    

    
  }

}
