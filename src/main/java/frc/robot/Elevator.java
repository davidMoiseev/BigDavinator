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
    Joystick HOTSticks = new Joystick(0);
    public void robotInit () {
        ELEVATOR1.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
        ELEVATOR2.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
        ELEVATOR1.reverseSensor(false);
        ELEVATOR2.reverseSensor(false);
        ELEVATOR1.configNominalOutputVoltage(+0.0f, -0.0f);
        ELEVATOR2.configNominalOutputVoltage(+0.0f, -0.0f);
        ELEVATOR1.configPeakOutputVoltage(+12.0f, -12.0f);
        ELEVATOR2.configPeakOutputVoltage(+12.0f, -12.0f);
        ELEVATOR1.setProfile(0);
        ELEVATOR2.setProfile(0);
        ELEVATOR1.setF(0);
        ELEVATOR2.setF(0);
        ELEVATOR1.setP(0);
        ELEVATOR2.setP(0);
        ELEVATOR1.setI(0);
        ELEVATOR2.setI(0);
        ELEVATOR1.setD(0);
        ELEVATOR2.setD(0);
        ELEVATOR1.setMotionMagicAcceleration(0);
        ELEVATOR2.setMotionMagicAcceleration(0);
        ELEVATOR1.setMotionMagicCruiseVelocity(0);
        ELEVATOR2.setMotionMagicCruiseVelocity(0);
    }
    public void teleopPeriodic () {
        double leftYstick = HOTSticks.getAxis(AxisType.kY);
        double motorOutput = ELEVATOR1.getOutputVoltage() / ELEVATOR1.getBusVoltage();
        double motorOutput = ELEVATOR2.getOutputVoltage() / ELEVATOR2.getBusVoltage();
        if (HOTSticks.getRawButton(1)) {
            double targetSpeed = leftYstick * 10.0;
        }
    }
}
