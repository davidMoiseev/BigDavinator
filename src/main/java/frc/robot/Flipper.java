/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.hotteam67.HotController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.constants.FlipperConstants;
import frc.robot.constants.ManipulatorSetPoint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */

public class Flipper extends MotionMagicActuator
{
    double output = 0.0;
    public static final int ANGLE_TO_TICKS = (int)((4096.0 / 360.0) * (50.0 / 18.0));
    private final double startingAngle;
    private final boolean inverted;


    public Flipper(int ID, boolean inverted, double startingAngle)
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

    public void setTarget(ManipulatorSetPoint targetPoint)
    {
        setTarget(targetPoint.frontFlipper * ANGLE_TO_TICKS);
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
        SmartDashboard.putNumber("FrontFlipper Power", primaryTalon.getMotorOutputPercent());
        SmartDashboard.putBoolean("FrontFlipper Reached", reachedTarget());
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

}
