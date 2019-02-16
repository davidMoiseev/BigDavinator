/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ElevatorConstants;

/**
 * Add your docs here.
 */

 public class Elevator extends MotionMagicActuator {

    public Elevator(TalonSRX primaryTalon, TalonSRX secondaryTalon) {
        super(primaryTalon, secondaryTalon);

        setNominalOutputForward(ElevatorConstants.nominalOutputForward);
        setNominalOutputReverse(ElevatorConstants.nominalOutputReverse);
        setPeakOutputForward(ElevatorConstants.peakOutputForward);
        setPeakOutputReverse(ElevatorConstants.peakOutputReverse);
        setMotionAcceleration(ElevatorConstants.motionAcceleration);
        setMotionCruiseVelocity(ElevatorConstants.motionCruiseVelocity);
        setTimeoutms(ElevatorConstants.timeoutms);
        setSensorPhase(ElevatorConstants.sensorPhase);

        setForwardSoftLimitThreshold(ElevatorConstants.forwardSoftLimitThreshold);
        setReverseSoftLimitThreshold(ElevatorConstants.reverseSoftLimitThreshold);

        SRX_PID_0.setSlotIdx(ElevatorConstants.slotIdx);
        SRX_PID_0.setPidIdx(ElevatorConstants.pidIdx);
        SRX_PID_0.setFeedForward(ElevatorConstants.feedForward);
        SRX_PID_0.setProportional(ElevatorConstants.proportional);
        SRX_PID_0.setDerivative(ElevatorConstants.derivative);
        SRX_PID_0.setIntegral(ElevatorConstants.integral);
        SRX_PID_0.setFeedbackDevice(ElevatorConstants.feedbackDevice);

    }

    @Override
    public void displaySensorsValue() {
        SmartDashboard.putNumber("Elevator Position ticks", GetSensorValue());
        SmartDashboard.putNumber("Elevator Position inches", GetSensorValue()*2.75*Math.PI/8192);
        SmartDashboard.putNumber("Elevator Power", primaryTalon.getMotorOutputPercent());
        SmartDashboard.putNumber("Elevator Error", primaryTalon.getClosedLoopError());
        SmartDashboard.putNumber("Elevator target", primaryTalon.getClosedLoopTarget());
    }

    @Override
    public void getError() {

    }
}