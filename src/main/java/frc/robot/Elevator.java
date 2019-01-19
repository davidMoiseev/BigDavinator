/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Add your docs here.
 */
public class Elevator {
    TalonSRX ELEVATOR1 = new TalonSRX(5);
    TalonSRX ELEVATOR2 = new TalonSRX(6);
        ELEVATOR1.selectProfileSlot(1, 2);
        ELEVATOR1.configNominalOutputForward(0, 100);
        ELEVATRO1.configNominalOutputReverse(0, 100);
        ELEVATOR1.configPeakOutputForward(1, 100);
        ELEVATOR1.configPeakOutputReverse(-1, 100);
        ELEVATOR1.configMotionCruiseVelocity(15000, 100);
        ELEVATOR.configMotionAcceleration(6700, 100);
        ELEVATOR2.selectProfileSlot(1, 2);
        ELEVATOR2.configNominalOutputForward(0, 100);
        ELEVATOR2.configNominalOutputReverse(0, 100);
        ELEVATOR2.configPeakOutputForward(1, 100);
        ELEVATOR2.configPeakOutputReverse(-1, 100);
        ELEVATOR2.configMotionCruiseVelocity(15000, 100);
        ELEVATOR2.configMotionAcceleration(6700, 100);
}