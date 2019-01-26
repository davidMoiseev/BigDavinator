/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Add your docs here.
 */
public class Elevator {
    static TalonSRX ELEVATOR1 = new TalonSRX(5);
    /*static TalonSRX ELEVATOR2 = new TalonSRX(6);*/
    public final static int TOP_AUTO = 1;
    public final static int LOW_AUTO = 2;
    public final static int TOP = 3;
    public final static int LOW = 4;

    public static void ElevatorINIT(){
        ELEVATOR1.selectProfileSlot(1, 2);
        ELEVATOR1.configNominalOutputForward(0, 100);
        ELEVATOR1.configNominalOutputReverse(0, 100);
        ELEVATOR1.configPeakOutputForward(1, 100);
        ELEVATOR1.configPeakOutputReverse(-1, 100);
        ELEVATOR1.configMotionCruiseVelocity(15000, 100);
        ELEVATOR1.configMotionAcceleration(6700, 100);
        /*ELEVATOR2.selectProfileSlot(1, 2);
        ELEVATOR2.configNominalOutputForward(0, 100);
        ELEVATOR2.configNominalOutputReverse(0, 100);
        ELEVATOR2.configPeakOutputForward(1, 100);
        ELEVATOR2.configPeakOutputReverse(-1, 100);
        ELEVATOR2.configMotionCruiseVelocity(15000, 100);
        ELEVATOR2.configMotionAcceleration(6700, 100);*/
    }
static Joystick HotJoystick = new Joystick(1);
private static double desiredElevatorPosition;
private static boolean firstElevator;

public static double ClosedLoopError(){
    return ELEVATOR1.getClosedLoopError(0);
}
public static boolean CurrentElevatorPosition() {
    if (Math.abs(ClosedLoopError()) < 40){
        return true;
    }else{
        return false;
    }
}
public static void MotionMagicElevator(){
    int AUTON_STATE = 1;
    switch(AUTON_STATE){
    case TOP_AUTO:
        desiredElevatorPosition = 3000;
        ELEVATOR1.set(ControlMode.MotionMagic, desiredElevatorPosition);
        /*ELEVATOR2.set(ControlMode.MotionMagic, desiredElevatorPosition);*/
        AUTON_STATE = AUTON_STATE + 1;
        break;
    case LOW_AUTO:
        desiredElevatorPosition = 1500;
        ELEVATOR1.set(ControlMode.MotionMagic, desiredElevatorPosition);
        /*ELEVATOR2.set(ControlMode.MotionMagic, desiredElevatorPosition);*/
        AUTON_STATE = AUTON_STATE + 1;
        break;
    case TOP:
        if (firstElevator == false){
            desiredElevatorPosition = 3000;
            firstElevator = true;
        }else{
            if(HotJoystick.getPOV() == 315 || HotJoystick.getPOV() == 0 || HotJoystick.getPOV() == 45){
                desiredElevatorPosition = desiredElevatorPosition + 250;
            } else if(HotJoystick.getPOV() == 225 || HotJoystick.getPOV() == 180 || HotJoystick.getPOV() == 135){
                desiredElevatorPosition = desiredElevatorPosition - 250;
            }
        }
            ELEVATOR1.set(ControlMode.MotionMagic, desiredElevatorPosition);
            /*ELEVATOR2.set(ControlMode.MotionMagic, desiredElevatorPosition);*/
            AUTON_STATE = AUTON_STATE + 1;
            break;
    case LOW:
        if (firstElevator == false){
            desiredElevatorPosition = 1500;
            firstElevator = true;
        }else{
            if(HotJoystick.getPOV() == 315 || HotJoystick.getPOV() == 0 || HotJoystick.getPOV() == 45){
                desiredElevatorPosition = desiredElevatorPosition + 250;
            }else if(HotJoystick.getPOV() == 225 || HotJoystick.getPOV() == 180 || HotJoystick.getPOV() == 135){
                desiredElevatorPosition = desiredElevatorPosition - 250;
            }
        }
        ELEVATOR1.set(ControlMode.MotionMagic, desiredElevatorPosition);
        /*ELEVATOR2.set(ControlMode.MotionMagic, desiredElevatorPosition);*/
        AUTON_STATE = AUTON_STATE + 1;
        break;
    }
}
    public static void ZeroElevator(){
        ELEVATOR1.setSelectedSensorPosition(0, 0, 0);
        /*ELEVATOR2.setSelectedSensorPosition(0, 0, 0);*/
    }
}