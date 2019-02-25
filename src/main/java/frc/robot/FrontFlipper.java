/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.hotteam67.HotController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.constants.FrontFlipperConstants;
import frc.robot.constants.ManipulatorSetPoint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */

public class FrontFlipper extends MotionMagicActuator
{
    double output = 0.0;
    public static final int ANGLE_TO_TICKS = 0;


    public FrontFlipper(int ID)
    {
        super(ID);
    }

    public void Update(HotController joystick)
    {
        primaryTalon.set(ControlMode.PercentOutput, .2);
    }

    @Override
    public void initialize()
    {
        super.initialize();
        primaryTalon.setSelectedSensorPosition(-20 * ANGLE_TO_TICKS);
    }

    /**
     * Set Motion Magic Target
     */
    public void setTarget(double setPoint)
    {
        super.setTarget(setPoint * ANGLE_TO_TICKS);
    }

    public void setTarget(ManipulatorSetPoint targetPoint)
    {
        setTarget(targetPoint.frontFlipper * ANGLE_TO_TICKS);
    }

    /**
     * Set angle in degrees
     * @param angle
     */
    public void setPosition(double angle)
    {
        primaryTalon.setSelectedSensorPosition((int)(angle * ANGLE_TO_TICKS));
    }
    

    @Override
    public void displaySensorsValue()
    {
        SmartDashboard.putNumber("FrontFlipper Position ticks", getSensorValue());
        SmartDashboard.putNumber("FrontFlipper Power", primaryTalon.getMotorOutputPercent());
        SmartDashboard.putBoolean("FrontFlipper Reached", reachedTarget());
    }

    @Override
    public boolean reachedTarget()
    {
        return super.reachedTarget((int)(1.0  * ANGLE_TO_TICKS), .07);
    }

    @Override
    public double getPosition()
    {
        return getSensorValue() * FrontFlipperConstants.TICKS_TO_INCHES;
    }

}
