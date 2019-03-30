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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.hotteam67.HotLogger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.IManipulatorSetPoint;
import frc.robot.constants.WiringIDs;

/**
 * Add your docs here.
 */

public class Elevator extends MotionMagicActuator
{

    public Elevator(TalonSRX primaryTalon, TalonSRX secondaryTalon)
    {
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
    public void initialize()
    {
        super.initialize();
        if (WiringIDs.IS_PRACTICE_BOT)
        {
            primaryTalon.setInverted(true);
            secondaryTalon.setInverted(true);
        }
        else
        {
            primaryTalon.setSensorPhase(false);
            secondaryTalon.setSensorPhase(false);
        }
    }

    private static void Log(String input, double value)
    {
        SmartDashboard.putNumber(input, value);
        HotLogger.Log(input, value);
    }

    @Override
    public void displaySensorsValue()
    {
        Log("Elevator Position ticks", getSensorValue());
        Log("Elevator Position inches", getPosition());
        Log("Elevator Power", primaryTalon.getMotorOutputPercent());
        Log("Elevator Error", getError());
        if (primaryTalon.getControlMode() == ControlMode.MotionMagic)
        {
            Log("Elevator target", primaryTalon.getClosedLoopTarget());
        }
        Log("Elevator Bus Voltage", primaryTalon.getBusVoltage());
        Log("Elevator Output Voltage", primaryTalon.getMotorOutputVoltage());
        Log("Elevator Current", primaryTalon.getOutputCurrent());
    }

    public static final List<String> LoggerTags = new ArrayList<>(
            Arrays.asList("Elevator Position ticks", "Elevator Position inches", "Elevator Power", "Elevator Error",
                    "Elevator target", "Elevator Bus Voltage", "Elevator Output Voltage", "Elevator Current"));

    @Override
    public void setTarget(double target)
    {
        super.setTarget(target / ElevatorConstants.TICKS_TO_INCHES);
    }

    public void setTarget(IManipulatorSetPoint targetPoint)
    {
        setTarget(targetPoint.elevatorHeight());
    }

    public boolean reachedTarget()
    {
        return super.reachedTarget(ElevatorConstants.allowableError, ElevatorConstants.minimumTimeToReachTarget);
    }

    @Override
    public double getPosition()
    {
        return getSensorValue() * ElevatorConstants.TICKS_TO_INCHES;
    }

	public void checkEncoder()
	{
        checkEncoder((int)(12.0 / ElevatorConstants.TICKS_TO_INCHES));
	}
}