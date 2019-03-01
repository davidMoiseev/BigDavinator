/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

/**
 * Add your docs here.
 */
public class ElevatorConstants
{
    public static final double nominalOutputForward = 0;
    public static final double nominalOutputReverse = 0;
    public static final int forwardSoftLimitThreshold = 30000;
    public static final int reverseSoftLimitThreshold = -10;
    public static final double peakOutputForward = 1;
    public static final double peakOutputReverse = -1;
    public static final int motionCruiseVelocity = 10000;
    public static final int motionAcceleration = 10000;
    public static final boolean sensorPhase = true;
    public static final int timeoutms = 100;
    public static final int slotIdx = 0;
    public static final int pidIdx = 0;
    public static final double feedForward = 0.2;
    // .45 was decent
    public static final double proportional = 0.45;// .71 initially
    public static final double derivative = 0.001 * 0;
    public static final double integral = 0.0020 * 0;// .0016 initially
    public static final FeedbackDevice feedbackDevice = FeedbackDevice.CTRE_MagEncoder_Relative;
    public static final double TICKS_TO_INCHES = 2.75 * Math.PI / 8192;
    public static final int allowableError = (int) (0.5 / TICKS_TO_INCHES);
    public static final double minimumTimeToReachTarget = 0.075;
}
