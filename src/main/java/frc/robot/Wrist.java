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

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;

import org.hotteam67.HotLogger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.IManipulatorSetPoint;
import frc.robot.constants.ManipulatorSetPoint;
import frc.robot.constants.WiringIDs;
import frc.robot.constants.WristConstants;

/**
 * Add your docs here.
 */
public class Wrist extends MotionMagicActuator
{
    private CANifier wristCan;

    public Wrist(int primaryCAN_ID/* , int secondaryCAN_ID */)
    {
        super(primaryCAN_ID/* , secondaryCAN_ID */);
        
        wristCan = new CANifier(WiringIDs.CANIFIER_WRIST);

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
    public void initialize()
    {
        super.initialize();

        primaryTalon.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, 0, 20);
        primaryTalon.configRemoteFeedbackFilter(WiringIDs.CANIFIER_WRIST, RemoteSensorSource.CANifier_Quadrature, 0);
    }

    private static void Log(String tag, double value)
    {
        SmartDashboard.putNumber(tag, value);
        HotLogger.Log(tag, value);
    }

    @Override
    public void displaySensorsValue()
    {
        Log("Wirst Position ticks", getSensorValue());
        Log("A Wirst Position degree", getPosition());
        SmartDashboard.putNumber("A WristCan Degree", - wristCan.getQuadraturePosition() * WristConstants.TICKS_TO_DEGREES);
        Log("Wirst Power", primaryTalon.getMotorOutputPercent());
        if (primaryTalon.getControlMode() == ControlMode.MotionMagic)
        {
            Log("Wirst Error", primaryTalon.getClosedLoopError());
            Log("Wirst target", primaryTalon.getClosedLoopTarget());
        }
        Log("Wrist Bus Voltage", primaryTalon.getBusVoltage());
        Log("Wrist Output Voltage", primaryTalon.getMotorOutputVoltage());
        Log("Wrist Current", primaryTalon.getOutputCurrent());
        Log("Wrist CANifier Voltage", wristCan.getBusVoltage());
    }

    public static final List<String> LoggerTags = new ArrayList<>(
            Arrays.asList("Wrist CANifier Voltage", "Wirst Position ticks", "A Wirst Position degree", "Wirst Power", "Wirst Error",
                    "Wirst target", "Wrist Bus Voltage", "Wrist Output Voltage", "Wrist Current"));

    public void setPosition(double angle)
    {
        wristCan.setQuadraturePosition((int) (angle / WristConstants.TICKS_TO_DEGREES), 100);
    }

    @Override
    public void zeroSensors()
    {
        wristCan.setQuadraturePosition(0, WristConstants.timeoutms);
    }

    @Override
    public double getPosition()
    {
        return -getSensorValue() * WristConstants.TICKS_TO_DEGREES;
    }

    @Override
    public void setTarget(double target)
    {
        super.setTarget(-target / WristConstants.TICKS_TO_DEGREES);
    }

    public void setTarget(IManipulatorSetPoint targetPoint)
    {
        setTarget(targetPoint.wristAngle());
    }

    public boolean reachedTarget()
    {
        return super.reachedTarget(WristConstants.allowableError, WristConstants.minimumTimeToReachTarget);
    }

    public void checkEncoder()
    {
        checkEncoder(50);
    }

}
