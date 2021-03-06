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
public class ArmConstants
{
    public static final double nominalOutputForward = 0;
    public static final double nominalOutputReverse = 0;
    public static final int forwardSoftLimitThreshold = 61910;
    public static final int reverseSoftLimitThreshold = -64130;
    public static final double peakOutputForward = 1;
    public static final double peakOutputReverse = -1;
    public static final int motionCruiseVelocity = 10000;
    public static final int motionAcceleration = 10000;
    public static final boolean sensorPhase = (!WiringIDs.IS_PRACTICE_BOT);
    public static final int timeoutms = 100;
    public static final int slotIdx = 0;
    public static final int pidIdx = 0;
    public static final double feedForward = 0.2378;
    public static final double proportional = 1.8;
    public static final double derivative = 0.2;
    public static final double integral = 0.0016 * 0;
    public static final FeedbackDevice feedbackDevice = FeedbackDevice.CTRE_MagEncoder_Relative;
    public static final double TICKS_TO_DEGREES = 0.0288 * 360 / 4096;
    public static final double length = 20;

    public static final int allowableError = (int) (7.5 / TICKS_TO_DEGREES);
    public static final double allowableErrorDegrees = allowableError * TICKS_TO_DEGREES;
    public static final double minimumTimeToReachTarget = 0.1;
}
