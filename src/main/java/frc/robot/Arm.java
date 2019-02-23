/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ManipulatorSetPoint;

/**
 * Add your docs here.
 */
public class Arm extends MotionMagicActuator {

    public Arm(int primaryCAN_ID/* , int secondaryCAN_ID */) {
        super(primaryCAN_ID/* , secondaryCAN_ID */);

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
        SmartDashboard.putNumber("Arm Position ticks", getSensorValue());
        SmartDashboard.putNumber("Arm Position degrees", getPosition());
        SmartDashboard.putNumber("Arm Power", primaryTalon.getMotorOutputPercent());
        if (primaryTalon.getControlMode() == ControlMode.MotionMagic) {
            SmartDashboard.putNumber("Arm Error", primaryTalon.getClosedLoopError());
            SmartDashboard.putNumber("Arm target", primaryTalon.getClosedLoopTarget());
        }
    }

    @Override
    public void setTarget(double target) {
        super.setTarget(-target / ArmConstants.TICKS_TO_DEGREES);
    }

    public void setTarget(ManipulatorSetPoint targetPoint) {
        setTarget(targetPoint.armAngle());
    }

    public void setPosition(double angle) {
        primaryTalon.setSelectedSensorPosition((int) (angle / ArmConstants.TICKS_TO_DEGREES));
    }

	public boolean reachedTarget() {
        return super.reachedTarget(ArmConstants.allowableError, ArmConstants.minimumTimeToReachTarget);
	}

    @Override
    public double getPosition() {
        return getSensorValue() * ArmConstants.TICKS_TO_DEGREES;
    }
}