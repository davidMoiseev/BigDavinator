/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * Add your docs here.
 */
public class SRX_PID {
    private int slotIdx;
    private int pidIdx;
    private double feedForward;
    private double proportional;
    private double derivative;
    private double integral;
    private FeedbackDevice feedbackDevice;
    private int timeoutms;

    public void initatlize(TalonSRX talon) {
        talon.selectProfileSlot(slotIdx, pidIdx);
        talon.config_kF(slotIdx, feedForward);
        talon.config_kP(slotIdx, proportional);
        talon.config_kD(slotIdx, derivative);
        talon.config_kI(slotIdx, integral);
        talon.configSelectedFeedbackSensor(feedbackDevice, slotIdx, timeoutms);
    }

    /**
     * @return the proportional
     */
    public double getProportional() {
        return proportional;
    }

    /**
     * @return the timeout
     */
    public int getTimeout() {
        return timeoutms;
    }

    /**
     * @param timeout the timeout to set
     */
    public void setTimeout(int timeoutms) {
        this.timeoutms = timeoutms;
    }

    /**
     * @return the feedbackDevice
     */
    public FeedbackDevice getFeedbackDevice() {
        return feedbackDevice;
    }

    /**
     * @param feedbackDevice the feedbackDevice to set
     */
    public void setFeedbackDevice(FeedbackDevice feedbackDevice) {
        this.feedbackDevice = feedbackDevice;
    }

    /**
     * @return the pidIdx
     */
    public int getPidIdx() {
        return pidIdx;
    }

    /**
     * @param pidIdx the pidIdx to set
     */
    public void setPidIdx(int pidIdx) {
        this.pidIdx = pidIdx;
    }

    /**
     * @return the integral
     */
    public double getIntegral() {
        return integral;
    }

    /**
     * @param integral the integral to set
     */
    public void setIntegral(double integral) {
        this.integral = integral;
    }

    /**
     * @return the derivative
     */
    public double getDerivative() {
        return derivative;
    }

    /**
     * @param derivative the derivative to set
     */
    public void setDerivative(double derivative) {
        this.derivative = derivative;
    }

    /**
     * @return the feedForward
     */
    public double getFeedForward() {
        return feedForward;
    }

    /**
     * @param feedForward the feedForward to set
     */
    public void setFeedForward(double feedForward) {
        this.feedForward = feedForward;
    }

    /**
     * @return the slotIdx
     */
    public int getSlotIdx() {
        return slotIdx;
    }

    /**
     * @param slotIdx the slotIdx to set
     */
    public void setSlotIdx(int slotIdx) {
        this.slotIdx = slotIdx;
    }

    /**
     * @param proportional the proportional to set
     */
    public void setProportional(double proportional) {
        this.proportional = proportional;
    }
}
