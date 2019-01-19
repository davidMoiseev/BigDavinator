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
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain {
    public static final int TALON_LF = 7;
    public static final int TALON_PIGEON = 6;
    public static final int TALON_RF = 4;
    public static final int TALON_RB = 3;
    public static final int TALON_LB = 6;
    public static final int TALON_LM = 5;
    public static final int TALON_RM = 4;
    public static final int TALON_H = 1;
    WPI_TalonSRX LFTalon = new WPI_TalonSRX(TALON_LF);
    WPI_TalonSRX RFTalon = new WPI_TalonSRX(TALON_RF);
    WPI_TalonSRX RBTalon = new WPI_TalonSRX(TALON_RB);
    WPI_TalonSRX LBTalon = new WPI_TalonSRX(TALON_LB);
    WPI_TalonSRX HTalon = new WPI_TalonSRX(TALON_H);
    WPI_TalonSRX LMTalon = new WPI_TalonSRX(TALON_LM);
    WPI_TalonSRX RMTalon = new WPI_TalonSRX(TALON_RM);
    public PigeonIMU pigeon = new PigeonIMU(TALON_PIGEON);

    HotSticks hotDrive = new HotSticks(0);
    HotSticks hotOp = new HotSticks(1);
    Solenoid solenoidH = new Solenoid(0);

    public double turn;
    public double h;
    public double forward;
    public double speedR;
    public double speedL;
    public double speedH;

    //Joystick Mapping
    public double x;
    public double y;

    LMTalon.follow(LFTalon);
    LBTalon.follow(LFTalon);
    RMTalon.follow(RFTalon);
    RBTalon.follow(RFTalon);

    public void Drivetrain(){
    
        LFTalon.setSensorPhase(true);

        // leftTalon.selectProfileSlot(0, 0);
        // rightTalon.selectProfileSlot(0, 0);
        
        LFTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        RFTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        HTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        
        LFTalon.setSelectedSensorPosition(0); 
        RFTalon.setSelectedSensorPosition(0);
        HTalon.setSelectedSensorPosition(0);
        
        LFTalon.set(ControlMode.PercentOutput, 0.0);
        RFTalon.set(ControlMode.PercentOutput, 0.0);
        HTalon.set(ControlMode.PercentOutput, 0.0);

        pigeon.setYaw(0);


    }

    public void dropH(boolean state){
        solenoidH.set(state);
    }

    public void driveManualTank(double kFwd, double kTurn, double kSpeed){
        hotDrive.setDeadBandLY(0.1);
        hotDrive.setDeadBandRX(0.1);
        double forward = hotDrive.getStickLY();
        double turn =  hotDrive.getStickRX();

        double speedR = -(forward * kFwd) + (turn * kTurn);
        double speedL = (forward * kFwd) + (turn * kTurn);

        if(speedR > 1){
            speedR = 1;
        }else if(speedL > 1){
            speedR = 1;
        }

        if((speedR <= 1) && (speedL <= 1)){
            RFTalon.set(ControlMode.PercentOutput, (speedR * kSpeed));
            LFTalon.set(ControlMode.PercentOutput, (speedL * kSpeed));
        }

    }
    
    public void driveManualH(double kFwd, double kTurn, double kSpeed, double kH){
        hotDrive.setDeadBandLY(0.1);
        hotDrive.setDeadBandRX(0.1);
        hotDrive.setDeadBandLX(0.1);
        turn =  hotDrive.getStickRX();
        h = hotDrive.getStickLX();
        forward = hotDrive.getStickLY();

        speedR = (forward * kFwd) - (turn * kTurn);
        speedL = (forward * kFwd) + (turn * kTurn);
        speedH = (h * kH);

        if(speedR > 1){
            speedR = 1;
        }else if(speedL > 1){
            speedL = 1;
        }

        // if(speedH > 0.05){
        //     this.dropH(true);
        // }else{
        //     this.dropH(false);
        // }

        if((speedR <= 1) && (speedL <= 1) && (speedH <= 1)){
            RFTalon.set(ControlMode.PercentOutput, (speedR * kSpeed));
            LFTalon.set(ControlMode.PercentOutput, (speedL * kSpeed));
            HTalon.set(ControlMode.PercentOutput, (speedH));
        }
    }

    public void writeDashboard(){
        SmartDashboard.putNumber("turn", turn);
        SmartDashboard.putNumber("forward", forward);
        SmartDashboard.putNumber("speedL", speedL);
        SmartDashboard.putNumber("speedR", speedR);
        SmartDashboard.putNumber("JoystickL", hotDrive.getStickLY());
        SmartDashboard.putNumber("JoystickR", hotDrive.getStickRX());
        }


}
