/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.hotteam67.HotLogger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.FlipperConstants;

/**
 * Add your docs here.
 */

public class Flipper
{
    double ANGLE_TO_TICKS = FlipperConstants.ANGLE_TO_TICKS;
    double target = 0;
    TalonSRX flipper;
    double allowableError = 4;
    double FLIPPER_P = FlipperConstants.proportional;

    public Flipper(int ID)
    {
        flipper = new TalonSRX(ID);
    }

    public void initialize()
    {
        setPosition(0);
    }

    public void control(double targetDegrees)
    {
        this.target = targetDegrees;
        if (!reachedTarget())
        {
            flipper.set(ControlMode.PercentOutput, FLIPPER_P * getError());
        }
        else
            flipper.set(ControlMode.PercentOutput, 0);
    }

    public void disable()
    {
        flipper.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Set angle in degrees
     * 
     * @param angle
     */
    public void setPosition(double angle)
    {
        flipper.setSelectedSensorPosition((int) (angle * ANGLE_TO_TICKS));
    }

    public double getError()
    {
        return target - getPosition();
    }

    public void displaySensorsValue()
    {
        SmartDashboard.putNumber("FrontFlipper Position", getPosition());
        HotLogger.Log("FrontFlipper Position", getPosition());
        SmartDashboard.putNumber("FrontFlipper Power", flipper.getMotorOutputPercent());
        HotLogger.Log("FrontFlipper Power", flipper.getMotorOutputPercent());
        SmartDashboard.putBoolean("FrontFlipper Reached", reachedTarget());
        HotLogger.Log("FrontFlipper Reached", reachedTarget() ? "True" : "False");
        SmartDashboard.putNumber("A Flipper Error", flipper.getClosedLoopError());
        SmartDashboard.putNumber("A flipper output", flipper.getMotorOutputPercent());
    }

    public boolean reachedTarget()
    {
        return Math.abs(getError()) < allowableError;
    }

    public double getPosition()
    {
        return flipper.getSelectedSensorPosition() / ANGLE_TO_TICKS;
    }

    public static final List<String> LoggerTags = new ArrayList<>(
            Arrays.asList("FrontFlipper Position Ticks", "FrontFlipper Power", "FrontFlipper Reached"));
}
