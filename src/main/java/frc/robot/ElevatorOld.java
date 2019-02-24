package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * List of PI Gains that work: P = .7; I = .0016 (Alright) P = .71; I = .0016
 * (Best)
 * 
 */
public class ElevatorOld
{
    TalonSRX ElevatorTalon = new TalonSRX(5);

    public void elevatorINIT()
    {
        ElevatorTalon.configFactoryDefault();
        ElevatorTalon.selectProfileSlot(0, 0);
        ElevatorTalon.config_kF(0, .2378);
        ElevatorTalon.config_kP(0, .71);
        ElevatorTalon.config_kI(0, .0016);
        ElevatorTalon.config_kD(0, 0);
        ElevatorTalon.configNominalOutputForward(0, 100);
        ElevatorTalon.configNominalOutputReverse(0, 100);
        ElevatorTalon.configForwardSoftLimitThreshold(MAX_LIMIT);
        ElevatorTalon.configReverseSoftLimitThreshold(MIN_LIMIT);
        ElevatorTalon.configPeakOutputForward(1, 100);
        ElevatorTalon.configPeakOutputReverse(-1, 100);
        ElevatorTalon.configMotionCruiseVelocity(10000, 100);
        ElevatorTalon.configMotionAcceleration(10000, 100);
        ElevatorTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
        ElevatorTalon.setSensorPhase(true);
    }

    public static int MAX_LIMIT = 20000;
    public static int MIN_LIMIT = 1000;
    public static double RAISE_CMD = 0.5;
    public static double LOWER_CMD = -0.5;
    public double TargetHeight = 0;

    public double position = 0;

    public double getElevatorPosition()
    {
        position = ElevatorTalon.getSelectedSensorPosition();
        return position;
    }

    public void getElevatorPower()
    {
        SmartDashboard.putNumber("Motor Power", ElevatorTalon.getMotorOutputPercent());
    }

    public void disableElevator()
    {
        ElevatorTalon.set(ControlMode.PercentOutput, 0);
    }

    public void zeroElevatorPosition()
    {
        ElevatorTalon.setSelectedSensorPosition(0);
    }

    public double getClosedLoopError()
    {
        return ElevatorTalon.getClosedLoopError(0);
    }

    public void raiseElevator()
    {
        if (ElevatorTalon.getSelectedSensorPosition() < MAX_LIMIT)
        {
            ElevatorTalon.set(ControlMode.PercentOutput, RAISE_CMD);
        }
        else
        {
            ElevatorTalon.set(ControlMode.PercentOutput, 0);
        }
    }

    public void lowerElevator()
    {
        if (ElevatorTalon.getSelectedSensorPosition() > MIN_LIMIT)
        {
            ElevatorTalon.set(ControlMode.PercentOutput, LOWER_CMD);
        }
        else
        {
            ElevatorTalon.set(ControlMode.PercentOutput, 0);
        }
    }

    public void motionMagicElevatorTop()
    {
        TargetHeight = 15000;
        ElevatorTalon.set(ControlMode.MotionMagic, TargetHeight);
    }

    public void motionMagicElevatorPrettyLow()
    {
        TargetHeight = 2000;
        ElevatorTalon.set(ControlMode.MotionMagic, TargetHeight);
    }

    public void motionMagicElevatorMid()
    {
        TargetHeight = 10000;
        ElevatorTalon.set(ControlMode.MotionMagic, TargetHeight);
    }

    public void motionMagicElevatorLow()
    {
        TargetHeight = 1000;
        ElevatorTalon.set(ControlMode.MotionMagic, TargetHeight);
    }

    public void zeroElevator()
    {
        ElevatorTalon.setSelectedSensorPosition(0, 0, 0);
    }
}