package org.hotteam67.motionmagic;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.hotteam67.HotLogger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class MotionMagicActuator implements IMotionMagicActuator
{

    protected TalonSRX primaryTalon;
    protected TalonSRX secondaryTalon;
    protected SRX_PID SRX_PID_0;
    private double nominalOutputForward;
    private double nominalOutputReverse;
    private int forwardSoftLimitThreshold;
    private int reverseSoftLimitThreshold;
    private double peakOutputForward;
    private double peakOutputReverse;
    private int motionCruiseVelocity;
    private int motionAcceleration;
    private boolean sensorPhase;
    private int timeoutms;
    private double reachedTargetTimer;
    protected int previousEncoderValue;
    protected int currentEncoderValue;

    public MotionMagicActuator(int primaryCAN_ID)
    {
        primaryTalon = new TalonSRX(primaryCAN_ID);
        SRX_PID_0 = new SRX_PID();
        secondaryTalon = null;
    }

    public MotionMagicActuator(TalonSRX primaryTalon)
    {
        this.primaryTalon = primaryTalon;
        SRX_PID_0 = new SRX_PID();
        this.secondaryTalon = null;
    }

    public MotionMagicActuator(int primaryCAN_ID, int secondaryCAN_ID)
    {
        primaryTalon = new TalonSRX(primaryCAN_ID);
        secondaryTalon = new TalonSRX(secondaryCAN_ID);
        SRX_PID_0 = new SRX_PID();
        secondaryTalon.set(ControlMode.Follower, primaryCAN_ID);
    }

    public MotionMagicActuator(TalonSRX primaryTalon, TalonSRX secondaryTalon)
    {
        this.primaryTalon = primaryTalon;
        this.secondaryTalon = secondaryTalon;
        SRX_PID_0 = new SRX_PID();
        secondaryTalon.set(ControlMode.Follower, this.primaryTalon.getDeviceID());
    }

    @Override
    public void initialize()
    {
        primaryTalon.configFactoryDefault();
        primaryTalon.configNominalOutputForward(nominalOutputForward, timeoutms);
        primaryTalon.configNominalOutputReverse(nominalOutputReverse, timeoutms);
        primaryTalon.configForwardSoftLimitThreshold(forwardSoftLimitThreshold);
        primaryTalon.configReverseSoftLimitThreshold(reverseSoftLimitThreshold);
        primaryTalon.configPeakOutputForward(peakOutputForward, timeoutms);
        primaryTalon.configPeakOutputReverse(peakOutputReverse, timeoutms);
        primaryTalon.configMotionAcceleration(motionAcceleration, timeoutms);
        primaryTalon.configMotionCruiseVelocity(motionCruiseVelocity, timeoutms);
        primaryTalon.setSensorPhase(sensorPhase);
        SRX_PID_0.initialize(primaryTalon);

        primaryTalon.set(ControlMode.PercentOutput, 0.0);

        if (secondaryTalon != null)
        {
            secondaryTalon.configFactoryDefault();
            secondaryTalon.configNominalOutputForward(nominalOutputForward, timeoutms);
            secondaryTalon.configNominalOutputReverse(nominalOutputReverse, timeoutms);
            secondaryTalon.configForwardSoftLimitThreshold(forwardSoftLimitThreshold);
            secondaryTalon.configReverseSoftLimitThreshold(reverseSoftLimitThreshold);
            secondaryTalon.configPeakOutputForward(peakOutputForward, timeoutms);
            secondaryTalon.configPeakOutputReverse(peakOutputReverse, timeoutms);
            secondaryTalon.configMotionAcceleration(motionAcceleration, timeoutms);
            secondaryTalon.configMotionCruiseVelocity(motionCruiseVelocity, timeoutms);
            secondaryTalon.setSensorPhase(sensorPhase);
            SRX_PID_0.initialize(secondaryTalon);
        }

    }

    protected boolean reachedTarget(int allowableError, double requiredTime)
    {
        boolean reached = false;

        if (Math.abs(getError()) <= allowableError)
        {
            if (reachedTargetTimer >= requiredTime)
            {
                reached = true;
            }
            reachedTargetTimer += 0.02;
        }
        else
        {
            reachedTargetTimer = 0;
        }
        return reached;
    }

    @Override
    public int getError()
    {
        if (primaryTalon.getControlMode() == ControlMode.MotionMagic)
        {
            return primaryTalon.getClosedLoopError();
        }
        else
        {
            return 2147483647;
        }
    }

    protected double getSensorValue()
    {
        return currentEncoderValue;
    }

    @Override
    public void zeroSensors()
    {
        previousEncoderValue = 0;
        primaryTalon.setSelectedSensorPosition(0);
    }

    @Override
    public void setTarget(double target)
    {
        primaryTalon.set(ControlMode.MotionMagic, target);
    }

    @Override
    public void disable()
    {
        primaryTalon.set(ControlMode.PercentOutput, 0);
    }

    public void manual(double Command)
    {
        primaryTalon.set(ControlMode.PercentOutput, Command);
    }

    /**
     * @return the timeoutms
     */
    public int getTimeoutms()
    {
        return timeoutms;
    }

    /**
     * @param timeoutms
     *                      the timeoutms to set
     */
    public void setTimeoutms(int timeoutms)
    {
        this.timeoutms = timeoutms;
    }

    /**
     * @return the sensorPhase
     */
    public boolean getSensorPhase()
    {
        return sensorPhase;
    }

    /**
     * @param sensorPhase
     *                        the sensorPhase to set
     */
    public void setSensorPhase(boolean sensorPhase)
    {
        this.sensorPhase = sensorPhase;
    }

    /**
     * @return the motionAcceleration
     */
    public double getMotionAcceleration()
    {
        return motionAcceleration;
    }

    /**
     * @param motionAcceleration
     *                               the motionAcceleration to set
     */
    public void setMotionAcceleration(int motionAcceleration)
    {
        this.motionAcceleration = motionAcceleration;
    }

    /**
     * @return the motionCruiseVelocity
     */
    public double getMotionCruiseVelocity()
    {
        return motionCruiseVelocity;
    }

    /**
     * @param motionCruiseVelocity
     *                                 the motionCruiseVelocity to set
     */
    public void setMotionCruiseVelocity(int motionCruiseVelocity)
    {
        this.motionCruiseVelocity = motionCruiseVelocity;
    }

    /**
     * @return the peakOutputReverse
     */
    public double getPeakOutputReverse()
    {
        return peakOutputReverse;
    }

    /**
     * @param peakOutputReverse
     *                              the peakOutputReverse to set
     */
    public void setPeakOutputReverse(double peakOutputReverse)
    {
        this.peakOutputReverse = peakOutputReverse;
    }

    /**
     * @return the peakOutputForward
     */
    public double getPeakOutputForward()
    {
        return peakOutputForward;
    }

    /**
     * @param peakOutputForward
     *                              the peakOutputForward to set
     */
    public void setPeakOutputForward(double peakOutputForward)
    {
        this.peakOutputForward = peakOutputForward;
    }

    /**
     * @return the reverseSoftLimitThreshold
     */
    public double getReverseSoftLimitThreshold()
    {
        return reverseSoftLimitThreshold;
    }

    /**
     * @param reverseSoftLimitThreshold
     *                                      the reverseSoftLimitThreshold to set
     */
    public void setReverseSoftLimitThreshold(int reverseSoftLimitThreshold)
    {
        this.reverseSoftLimitThreshold = reverseSoftLimitThreshold;
    }

    /**
     * @return the forwardSoftLimitThreshold
     */
    public double getForwardSoftLimitThreshold()
    {
        return forwardSoftLimitThreshold;
    }

    /**
     * @param forwardSoftLimitThreshold
     *                                      the forwardSoftLimitThreshold to set
     */
    public void setForwardSoftLimitThreshold(int forwardSoftLimitThreshold)
    {
        this.forwardSoftLimitThreshold = forwardSoftLimitThreshold;
    }

    /**
     * @return the nominalOutputReverse
     */
    public double getNominalOutputReverse()
    {
        return nominalOutputReverse;
    }

    /**
     * @param nominalOutputReverse
     *                                 the nominalOutputReverse to set
     */
    public void setNominalOutputReverse(double nominalOutputReverse)
    {
        this.nominalOutputReverse = nominalOutputReverse;
    }

    /**
     * @return the nominalOutputForward
     */
    public double getNominalOutputForward()
    {
        return nominalOutputForward;
    }

    /**
     * @param nominalOutputForward
     *                                 the nominalOutputForward to set
     */
    public void setNominalOutputForward(double nominalOutputForward)
    {
        this.nominalOutputForward = nominalOutputForward;
    }

    public void Update(int encoderValueChangeLimit)
    {
        if (primaryTalon.hasResetOccurred())
        {
            // SmartDashboard.putBoolean("Has Reset Occured", true);
            HotLogger.Log("Has Reset Occured", 1);
            currentEncoderValue = currentEncoderValue + previousEncoderValue;
            primaryTalon.setSelectedSensorPosition(currentEncoderValue);
        }
        else
        {
            currentEncoderValue = primaryTalon.getSelectedSensorPosition();
            if (Math.abs(currentEncoderValue - previousEncoderValue) > encoderValueChangeLimit)
            {
                //currentEncoderValue += previousEncoderValue;
                //primaryTalon.setSelectedSensorPosition(currentEncoderValue);
                HotLogger.Log("ENCODER JUMP HAPPENED", true);
            }
        }
        previousEncoderValue = currentEncoderValue;
    }
}
