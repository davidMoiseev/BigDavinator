/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.manipulator;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;

import org.hotteam67.HotLogger;
import org.hotteam67.motionmagic.MotionMagicActuator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.IManipulatorSetPoint;
import frc.robot.constants.WiringIDs;

/**
 * Add your docs here.
 */
public class Arm extends MotionMagicActuator
{

    private CANifier armCan;

    public Arm(int primaryCAN_ID/* , int secondaryCAN_ID */)
    {
        super(primaryCAN_ID/* , secondaryCAN_ID */);

        if (WiringIDs.IS_PRACTICE_BOT)
            armCan = new CANifier(WiringIDs.CANIFIER_ARM);

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
    public void zeroSensors()
    {
        if (WiringIDs.IS_PRACTICE_BOT)
            armCan.setQuadraturePosition(0, ArmConstants.timeoutms);
        else
            primaryTalon.setSelectedSensorPosition(0);
    }

    @Override
    public void initialize()
    {
        super.initialize();

        if (WiringIDs.IS_PRACTICE_BOT)
        {
            primaryTalon.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, 0, 20);
            primaryTalon.configRemoteFeedbackFilter(WiringIDs.CANIFIER_ARM, RemoteSensorSource.CANifier_Quadrature, 0);
        }

        primaryTalon.configPeakCurrentDuration(0);
        primaryTalon.configPeakCurrentLimit(30);
        primaryTalon.configContinuousCurrentLimit(25);
        primaryTalon.enableCurrentLimit(true);
    }

    private static void Log(String input, double value)
    {
        SmartDashboard.putNumber(input, value);
        HotLogger.Log(input, value);
    }

    @Override
    public void displaySensorsValue()
    {
        Log("Arm Position ticks", getSensorValue());
        Log("Arm Position degrees", getPosition());
        if (WiringIDs.IS_PRACTICE_BOT)
            SmartDashboard.putNumber("ARM CAN DEGREE", -armCan.getQuadraturePosition() * ArmConstants.TICKS_TO_DEGREES);
        Log("Arm Power", primaryTalon.getMotorOutputPercent());
        if (primaryTalon.getControlMode() == ControlMode.MotionMagic)
        {
            Log("Arm Error", primaryTalon.getClosedLoopError());
            Log("Arm target", primaryTalon.getClosedLoopTarget());
        }
        Log("Arm Bus Voltage", primaryTalon.getBusVoltage());
        Log("Arm Output Voltage", primaryTalon.getMotorOutputVoltage());
        Log("Arm Current", primaryTalon.getOutputCurrent());
        if (WiringIDs.IS_PRACTICE_BOT)
            Log("Arm CANifier Voltage", armCan.getBusVoltage());
    }

    public static final List<String> LoggerTags = new ArrayList<>(
            Arrays.asList("Arm CANifier Voltage", "Arm Position ticks", "Arm Position degrees", "Arm Power",
                    "Arm Error", "Arm target", "Arm Bus Voltage", "Arm Output Voltage", "Arm Current"));

    @Override
    public void setTarget(double target)
    {
        super.setTarget(-target / ArmConstants.TICKS_TO_DEGREES);
    }

    public void setTarget(IManipulatorSetPoint targetPoint)
    {
        setTarget(targetPoint.armAngle());
    }

    public void setPosition(double angle)
    {
        previousEncoderValue = (int) (angle / ArmConstants.TICKS_TO_DEGREES);
        if (WiringIDs.IS_PRACTICE_BOT)
            armCan.setQuadraturePosition(previousEncoderValue, 100);
        else
            primaryTalon.setSelectedSensorPosition(0);
    }

    public boolean reachedTarget()
    {
        return super.reachedTarget(ArmConstants.allowableError, ArmConstants.minimumTimeToReachTarget);
    }

    @Override
    public double getPosition()
    {
        return -getSensorValue() * ArmConstants.TICKS_TO_DEGREES;
    }

    public void checkEncoder()
    {
        checkEncoder((int) (25.0 / ArmConstants.TICKS_TO_DEGREES));
    }
}
