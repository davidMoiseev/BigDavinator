package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class FlipperConstants
{
    public static final double CARRY_BACK = 60;
    public static final double HATCH_BACK = 182;

    public static final double CARRY_FRONT = 25;
    public static final double HATCH_FRONT = 148;

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
    public static final double feedForward = 0.6;
    // .45 was decent
    public static final double proportional = 0.015;// .71 initially
    public static final double derivative = 0.001 * 0;
    public static final double integral = 0.0020 * 0;// .0016 initially
    public static final FeedbackDevice feedbackDevice = FeedbackDevice.CTRE_MagEncoder_Relative;
    public static final double ANGLE_TO_TICKS = 2100.0 / 90.0;
    public static final int allowableError = (int) (4 * ANGLE_TO_TICKS);
    public static final double minimumTimeToReachTarget = 0.075;
	public static double rampRate = .08;
}