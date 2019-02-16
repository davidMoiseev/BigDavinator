/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.WristConstants;

/**
 * Add your docs here.
 */
public class Wrist extends MotionMagicActuator {
    public Wrist(int primaryCAN_ID/*, int secondaryCAN_ID*/) {        
        super(primaryCAN_ID/*, secondaryCAN_ID*/);

            setNominalOutputForward(WristConstants.nominalOutputForward);
            setNominalOutputReverse(WristConstants.nominalOutputReverse);
            setPeakOutputForward(WristConstants.peakOutputForward);
            setPeakOutputReverse(WristConstants.peakOutputReverse);
            setMotionAcceleration(WristConstants.motionAcceleration);
            setMotionCruiseVelocity(WristConstants.motionCruiseVelocity);
            setTimeoutms(WristConstants.timeoutms);
            setSensorPhase(WristConstants.sensorPhase);

            setForwardSoftLimitThreshold(WristConstants.forwardSoftLimitThreshold);
            setReverseSoftLimitThreshold(WristConstants.reverseSoftLimitThreshold);

            SRX_PID_0.setSlotIdx(WristConstants.slotIdx);
            SRX_PID_0.setPidIdx(WristConstants.pidIdx);
            SRX_PID_0.setFeedForward(WristConstants.feedForward);
            SRX_PID_0.setProportional(WristConstants.proportional);
            SRX_PID_0.setDerivative(WristConstants.derivative);
            SRX_PID_0.setIntegral(WristConstants.integral);
            SRX_PID_0.setFeedbackDevice(WristConstants.feedbackDevice);

    }

        @Override
        public void displaySensorsValue() {
            SmartDashboard.putNumber("Wirst Position ticks", GetSensorValue());
            SmartDashboard.putNumber("Wirst Position degree", GetSensorValue() * 0.12 * 360/4096);
            SmartDashboard.putNumber("Wirst Power", primaryTalon.getMotorOutputPercent());
            SmartDashboard.putNumber("Wirst Error", primaryTalon.getClosedLoopError());
            SmartDashboard.putNumber("Wirst target", primaryTalon.getClosedLoopTarget());
    }

        @Override
        public void getError() {

    }
}