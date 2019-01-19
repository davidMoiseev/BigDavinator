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
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain {
    public static final int TALON_LF = 7;
    public static final int TALON_PIGEON = 6;
    public static final int TALON_RF = 2;
    public static final int TALON_RB = 3;
    public static final int TALON_LB = 6;
    public static final int TALON_LM = 5;
    public static final int TALON_RM = 4;
    public static final int TALON_H = 1;
    public WPI_TalonSRX LFTalon = new WPI_TalonSRX(TALON_LF);
    public WPI_TalonSRX RFTalon = new WPI_TalonSRX(TALON_RF);
    public WPI_TalonSRX RBTalon = new WPI_TalonSRX(TALON_RB);
    public WPI_TalonSRX LBTalon = new WPI_TalonSRX(TALON_LB);
    public WPI_TalonSRX HTalon = new WPI_TalonSRX(TALON_H);
    public WPI_TalonSRX LMTalon = new WPI_TalonSRX(TALON_LM);
    public WPI_TalonSRX RMTalon = new WPI_TalonSRX(TALON_RM);
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

    public boolean isHDown = false;
    public boolean stateA;
    public boolean prevStateA = false;

    public boolean stateStart;
    public boolean prevStateStart = false;


    public void Drivetrain(){
        LFTalon.configFactoryDefault();
        LMTalon.configFactoryDefault();
        LBTalon.configFactoryDefault();
        RFTalon.configFactoryDefault();
        RMTalon.configFactoryDefault();
        RBTalon.configFactoryDefault();

        LFTalon.setSensorPhase(true);
        LMTalon.setSensorPhase(true);
        LBTalon.setSensorPhase(true);

        // leftTalon.selectProfileSlot(0, 0);
        // rightTalon.selectProfileSlot(0, 0);
        
        LFTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        RFTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        RMTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        RBTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        LMTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        LBTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        HTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        
        LFTalon.setSelectedSensorPosition(0); 
        RFTalon.setSelectedSensorPosition(0);
        RMTalon.setSelectedSensorPosition(0);
        RBTalon.setSelectedSensorPosition(0);
        LBTalon.setSelectedSensorPosition(0);
        LMTalon.setSelectedSensorPosition(0);
        HTalon.setSelectedSensorPosition(0);
        
        LFTalon.set(ControlMode.PercentOutput, 0.0);
        RFTalon.set(ControlMode.PercentOutput, 0.0);
        RMTalon.set(ControlMode.PercentOutput, 0.0);
        RBTalon.set(ControlMode.PercentOutput, 0.0);
        LMTalon.set(ControlMode.PercentOutput, 0.0);
        LBTalon.set(ControlMode.PercentOutput, 0.0);
        HTalon.set(ControlMode.PercentOutput, 0.0);

        LFTalon.setNeutralMode(NeutralMode.Brake);
        RFTalon.setNeutralMode(NeutralMode.Brake);
        LMTalon.setNeutralMode(NeutralMode.Brake);
        RMTalon.setNeutralMode(NeutralMode.Brake);
        RBTalon.setNeutralMode(NeutralMode.Brake);
        LBTalon.setNeutralMode(NeutralMode.Brake);
        HTalon.setNeutralMode(NeutralMode.Brake);

        pigeon.setYaw(0);
        
        // LMTalon.follow(LFTalon);
        // LBTalon.follow(LFTalon);
        // RMTalon.follow(RFTalon);
        // RBTalon.follow(RFTalon);


    }

    public void dropH(boolean state){
        solenoidH.set(state);
    }

    // public boolean buttonDropH(){
    //     stateA = hotDrive.getButtonA();
    //     if((stateA != prevStateA) && (stateA == false) && (isHDown == false)){
    //         this.dropH(true);
    //         isHDown = true;
    //         prevStateA = stateA;
    //         return true;
    //     }else if((stateA != prevStateA) && (stateA == false) && (isHDown == true)){
    //         this.dropH(false);
    //         isHDown = false;
    //         prevStateA = stateA;
    //         return true;
    //     }else{
    //         prevStateA = stateA;
    //         return false;
    //     }
    // }

    // public boolean dropHOverride(){  
    //     if(this.buttonDropH() == true){
    //        return true; 
    //     }else{
    //         return false;
    //     }
    // }
    
    public void driveManualH(double kFwd, double kTurn, double kSpeed, double kH){
        hotDrive.setDeadBandLY(0.1);
        hotDrive.setDeadBandRX(0.1);
        hotDrive.setDeadBandLX(0.1);
        turn =  hotDrive.getStickRX();
        h = hotDrive.getStickLX();
        forward = hotDrive.getStickLY();

        double speedR = (forward * kFwd) + (turn * kTurn);
        double speedL = -(forward * kFwd) + (turn * kTurn);
        speedH = (h * kH);

        if(speedR > 1){
            speedR = 1;
        }else if(speedL > 1){
            speedL = 1;
        }

        if(hotDrive.getButtonA() == true){
            this.dropH(true);
        }else if(Math.abs(speedH) > 0.1){
            this.dropH(true);
        }else{
            this.dropH(false);
        }

        if((speedR <= 1) && (speedL <= 1) && (speedH <= 1)){
            RFTalon.set(ControlMode.PercentOutput, (speedR * kSpeed));
            RMTalon.set(ControlMode.PercentOutput, (speedR * kSpeed));
            RBTalon.set(ControlMode.PercentOutput, (speedR * kSpeed));
            LFTalon.set(ControlMode.PercentOutput, (speedL * kSpeed));
            LMTalon.set(ControlMode.PercentOutput, (speedL * kSpeed));
            LBTalon.set(ControlMode.PercentOutput, (speedL * kSpeed));
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
