/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ArmConstants;

/**
 * Add your docs here.
 */
public class Arm extends MotionMagicActuator {
    
    public Arm(int primaryCAN_ID/*, int secondaryCAN_ID*/) {
        super(primaryCAN_ID/*, secondaryCAN_ID*/);

        setNominalOutputForward(ArmConstants.nominalOutputForward);
        setNominalOutputReverse(ArmConstants.nominalOutputReverse);
        setPeakOutputForward(ArmConstants.peakOutputForward);
        setPeakOutputReverse(ArmConstants.peakOutputReverse);
        setMotionAcceleration(ArmConstants.motionAcceleration);
        setMotionCruiseVelocity(ArmConstants.motionCruiseVelocity);
        setTimeoutms(ArmConstants.timeoutms);
        setSensorPhase(ArmConstants.sensorPhase);

        setForwardSoftLimitThreshold(ArmConstants.forwardSoftLimitThreshold);
        setReverseSoftLimitThreshold(ArmConstants.reverseSoftLimitThreshold);

        SRX_PID_0.setSlotIdx(ArmConstants.slotIdx);
        SRX_PID_0.setPidIdx(ArmConstants.pidIdx);
        SRX_PID_0.setFeedForward(ArmConstants.feedForward);
        SRX_PID_0.setProportional(ArmConstants.proportional);
        SRX_PID_0.setDerivative(ArmConstants.derivative);
        SRX_PID_0.setIntegral(ArmConstants.integral);
        SRX_PID_0.setFeedbackDevice(ArmConstants.feedbackDevice);

    }

    @Override
    public void displaySensorsValue() {
        SmartDashboard.putNumber("Arm Position ticks", GetSensorValue());
        SmartDashboard.putNumber("Arm Position degrees", GetSensorValue() *360/(4096*34.72222));
        SmartDashboard.putNumber("Arm Power", primaryTalon.getMotorOutputPercent());
        SmartDashboard.putNumber("Arm Error", primaryTalon.getClosedLoopError());
        SmartDashboard.putNumber("Arm target", primaryTalon.getClosedLoopTarget());
    }

    @Override
    public void getError() {

    }
}