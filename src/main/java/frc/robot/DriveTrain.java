/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/*
driveManual
init 5 motors
*/
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain {
    public static final int TALON_LF = 100;
    public static final int TALON_PIGEON = 200;
    public static final int TALON_RF = 400;
    public static final int TALON_RB = 67;
    public static final int TALON_LB = 33;
    public static final int TALON_H = 3707;


    WPI_TalonSRX LFTalon = new WPI_TalonSRX(TALON_LF);
    WPI_TalonSRX RFTalon = new WPI_TalonSRX(TALON_RF);
    WPI_TalonSRX RBTalon = new WPI_TalonSRX(TALON_RB);
    WPI_TalonSRX LBTalon = new WPI_TalonSRX(TALON_LB);
    WPI_TalonSRX HTalon = new WPI_TalonSRX(TALON_H);
    public PigeonIMU pigeon = new PigeonIMU(TALON_PIGEON);

    Joystick joystick;

    public void Drivetrain(){
        LFTalon.setSensorPhase(true);
        LBTalon.setSensorPhase(true);

        
        // leftTalon.selectProfileSlot(0, 0);
        // rightTalon.selectProfileSlot(0, 0);
        
        LFTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        RFTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        LBTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        RBTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        HTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        
        LFTalon.setSelectedSensorPosition(0); 
        RFTalon.setSelectedSensorPosition(0);
        LBTalon.setSelectedSensorPosition(0); 
        RBTalon.setSelectedSensorPosition(0);
        HTalon.setSelectedSensorPosition(0);
        
        LFTalon.set(ControlMode.PercentOutput, 0.0);
        RFTalon.set(ControlMode.PercentOutput, 0.0);
        LBTalon.set(ControlMode.PercentOutput, 0.0);
        RBTalon.set(ControlMode.PercentOutput, 0.0);
        HTalon.set(ControlMode.PercentOutput, 0.0);
        

        pigeon.setYaw(0);
    }

    public void driveManualTank(double kFwd, double kTurn, double kSpeed){

        double forward = /*HotSticks.DriverLY()*/ joystick.getRawAxis(1);
        double turn = /*HotSticks.DriverRX()*/ joystick.getRawAxis(4);

        double speedR = (forward * kFwd) - (turn * kTurn);
        double speedL = (forward * kFwd) + (turn * kTurn);

        if(speedR > 1){
            speedR = 1;
        }else if(speedL > 1){
            speedR = 1;
        }

        if((speedR <= 1) && (speedL <= 1)){
            RFTalon.set(ControlMode.PercentOutput, (speedR * kSpeed));
            RBTalon.set(ControlMode.PercentOutput, (speedR * kSpeed));
            LFTalon.set(ControlMode.PercentOutput, (speedR * kSpeed));
            LBTalon.set(ControlMode.PercentOutput, (speedR * kSpeed));
        }

    }
    
    public void driveManualH(double kFwd, double kTurn, double kSpeed, double kH){

        double forward = /*HotSticks.DriverLY()*/ joystick.getRawAxis(1);
        double turn = /*HotSticks.DriverRX()*/ joystick.getRawAxis(4);
        double h = /*HotSticks.DriverLX()*/ joystick.getRawAxis(0);

        double speedR = (forward * kFwd) - (turn * kTurn);
        double speedL = (forward * kFwd) + (turn * kTurn);
        double speedH = (h * kH);

        if(speedR > 1){
            speedR = 1;
        }else if(speedL > 1){
            speedL = 1;
        }

        if((speedR <= 1) && (speedL <= 1) && (speedH <= 1)){
            RFTalon.set(ControlMode.PercentOutput, (speedR * kSpeed));
            RBTalon.set(ControlMode.PercentOutput, (speedR * kSpeed));
            LFTalon.set(ControlMode.PercentOutput, (speedR * kSpeed));
            LBTalon.set(ControlMode.PercentOutput, (speedR * kSpeed));
            HTalon.set(ControlMode.PercentOutput, (speedH));
        }

    }

}
