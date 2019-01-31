package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class MotionMagicActuator implements IMotionMagicActuator {

    private TalonSRX PrimaryTalon;
    private TalonSRX SecondaryTalon;
    
    public MotionMagicActuator(int primaryCAN_ID) {
        PrimaryTalon = new TalonSRX(primaryCAN_ID);
        SecondaryTalon = null;
    }

    public MotionMagicActuator(int primaryCAN_ID, int secondaryCAN_ID) {
        PrimaryTalon = new TalonSRX(primaryCAN_ID);
        SecondaryTalon = new TalonSRX(secondaryCAN_ID);
        SecondaryTalon.set(ControlMode.Follower, primaryCAN_ID);
    }

    protected double GetSensorValue(){
        return PrimaryTalon.getSelectedSensorPosition();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void getError() {
        return PrimaryTalon.getClosedLoopError();
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void disable() {

    }
}