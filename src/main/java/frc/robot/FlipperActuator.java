package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.FlipperConstants;

public class FlipperActuator extends MotionMagicActuator
{
    public FlipperActuator(int id)
    {
        super(id);

        setNominalOutputForward(FlipperConstants.nominalOutputForward);
        setNominalOutputReverse(FlipperConstants.nominalOutputReverse);
        setPeakOutputForward(FlipperConstants.peakOutputForward);
        setPeakOutputReverse(FlipperConstants.peakOutputReverse);
        setMotionAcceleration(FlipperConstants.motionAcceleration);
        setMotionCruiseVelocity(FlipperConstants.motionCruiseVelocity);
        setTimeoutms(FlipperConstants.timeoutms);
        setSensorPhase(FlipperConstants.sensorPhase);
        setClosedLoopRampRate(FlipperConstants.rampRate);

        setForwardSoftLimitThreshold(FlipperConstants.forwardSoftLimitThreshold);
        setReverseSoftLimitThreshold(FlipperConstants.reverseSoftLimitThreshold);

        SRX_PID_0.setSlotIdx(FlipperConstants.slotIdx);
        SRX_PID_0.setPidIdx(FlipperConstants.pidIdx);
        SRX_PID_0.setFeedForward(FlipperConstants.feedForward);
        SRX_PID_0.setProportional(FlipperConstants.proportional);
        SRX_PID_0.setDerivative(FlipperConstants.derivative);
        SRX_PID_0.setIntegral(FlipperConstants.integral);
        SRX_PID_0.setFeedbackDevice(FlipperConstants.feedbackDevice);
    }

    @Override
    public void initialize()
    {
        super.initialize();

        primaryTalon.setSelectedSensorPosition(0);
    }

    @Override
    public void displaySensorsValue()
    {
        String name = "Flipper " + primaryTalon.getDeviceID();
        SmartDashboard.putNumber(name + " Position", getPosition());
        SmartDashboard.putNumber(name + " Output", primaryTalon.getMotorOutputPercent());
        SmartDashboard.putBoolean(name + " Reached", reachedTarget());
    }

    @Override
    public boolean reachedTarget()
    {
        return Math.abs(primaryTalon.getClosedLoopError()) < FlipperConstants.allowableError;
    }

    public void setPosition(double angle)
    {
        primaryTalon.setSelectedSensorPosition((int)(FlipperConstants.ANGLE_TO_TICKS * angle));
    }

    @Override
    public double getPosition()
    {
        return getSensorValue() / FlipperConstants.ANGLE_TO_TICKS;
    }

    public void checkEncoder()
    {
        checkEncoder(50);
    }
}