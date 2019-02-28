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

import org.hotteam67.HotController;
import org.hotteam67.HotLogger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.FlipperConstants;
import frc.robot.constants.IManipulatorSetPoint;
import frc.robot.constants.ManipulatorSetPoint;

/**
 * Add your docs here.
 */

public class Flipper extends MotionMagicActuator
{
    double output = 0.0;
    public static final int ANGLE_TO_TICKS = (int) ((4096.0 / 360.0) * (50.0 / 18.0));
    private final double startingAngle;
    private final boolean inverted;
    private final boolean isBack;


    public Flipper(int ID, boolean inverted, double startingAngle, boolean isBack)
    {
        super(ID);

        setNominalOutputForward(FlipperConstants.nominalOutputForward);
        setNominalOutputReverse(FlipperConstants.nominalOutputReverse);
        setPeakOutputForward(FlipperConstants.peakOutputForward);
        setPeakOutputReverse(FlipperConstants.peakOutputReverse);
        setMotionAcceleration(FlipperConstants.motionAcceleration);
        setMotionCruiseVelocity(FlipperConstants.motionCruiseVelocity);
        setTimeoutms(FlipperConstants.timeoutms);
        setSensorPhase(FlipperConstants.sensorPhase);

        setForwardSoftLimitThreshold(FlipperConstants.forwardSoftLimitThreshold);
        setReverseSoftLimitThreshold(FlipperConstants.reverseSoftLimitThreshold);

        SRX_PID_0.setSlotIdx(FlipperConstants.slotIdx);
        SRX_PID_0.setPidIdx(FlipperConstants.pidIdx);
        SRX_PID_0.setFeedForward(FlipperConstants.feedForward);
        SRX_PID_0.setProportional(FlipperConstants.proportional);
        SRX_PID_0.setDerivative(FlipperConstants.derivative);
        SRX_PID_0.setIntegral(FlipperConstants.integral);
        SRX_PID_0.setFeedbackDevice(FlipperConstants.feedbackDevice);

        this.startingAngle = startingAngle;
        this.inverted = inverted;
        this.isBack = isBack;
    }

    public void Update(HotController joystick)
    {
        primaryTalon.set(ControlMode.PercentOutput, .2);
    }

    @Override
    public void initialize()
    {
        super.initialize();
        primaryTalon.setInverted(inverted);
        primaryTalon.setSelectedSensorPosition((int)(startingAngle * ANGLE_TO_TICKS));
    }

    /**
     * Set Motion Magic Target
     */
    public void setTarget(double setPoint)
    {
        super.setTarget(setPoint * ANGLE_TO_TICKS);
    }

    public void setTarget(IManipulatorSetPoint targetPoint)
    {
        double target = isBack ? targetPoint.backFlipper() : targetPoint.frontFlipper();
        setTarget(target * ANGLE_TO_TICKS);
    }

    /**
     * Set angle in degrees
     * @param angle
     */
    public void setPosition(double angle)
    {
        primaryTalon.setSelectedSensorPosition((int)(angle * ANGLE_TO_TICKS));
    }
    

    @Override
    public void displaySensorsValue()
    {
        SmartDashboard.putNumber("FrontFlipper Position ticks", getSensorValue());
        HotLogger.Log("FrontFlipper Position ticks", getSensorValue());
        SmartDashboard.putNumber("FrontFlipper Power", primaryTalon.getMotorOutputPercent());
        HotLogger.Log("FrontFlipper Power", primaryTalon.getMotorOutputPercent());
        SmartDashboard.putBoolean("FrontFlipper Reached", reachedTarget());
        HotLogger.Log("FrontFlipper Reached", reachedTarget() ? "True" : "False");
    }

    @Override
    public boolean reachedTarget()
    {
        return super.reachedTarget((int)(1.0  * ANGLE_TO_TICKS), .07);
    }

    @Override
    public double getPosition()
    {
        return getSensorValue() * FlipperConstants.TICKS_TO_INCHES;
    }

    public static final List<String> LoggerTags = new ArrayList<>(Arrays.asList("FrontFlipper Position Ticks", "FrontFlipper Power", "FrontFlipper Reached"));
}
