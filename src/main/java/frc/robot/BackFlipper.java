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
//import com.ctre.phoenix.motorcfontrol.can.WPI_TalonSRX;
import frc.robot.constants.BackFlipperConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */

public class BackFlipper extends MotionMagicActuator {
    TalonSRX BackFlipper = new TalonSRX(15);
    TalonSRX BackFlipperBack = new TalonSRX(16);
    double output = 0.0;
    
public BackFlipper(TalonSRX BackFlipper, TalonSRX BackFlipperBack)
{
    super(BackFlipper, BackFlipperBack);
    SRX_PID_0.setSlotIdx(BackFlipperConstants.slotIdx);
    SRX_PID_0.setPidIdx(BackFlipperConstants.pidIdx);
    SRX_PID_0.setFeedForward(BackFlipperConstants.feedForward);
    SRX_PID_0.setProportional(BackFlipperConstants.proportional);
    SRX_PID_0.setDerivative(BackFlipperConstants.derivative);
    SRX_PID_0.setIntegral(BackFlipperConstants.integral);
    SRX_PID_0.setFeedbackDevice(BackFlipperConstants.feedbackDevice);

}
public void Update(HotController joystick)
	{
        BackFlipper.set(ControlMode.PercentOutput, BackFlipperOutput(joystick));
        BackFlipperBack.set(ControlMode.PercentOutput, -BackFlipperOutput(joystick));
    }

public double BackFlipperOutput(HotController joystick)
    {
        int flipperstate = 0;
        double flipperoutput = 0.0;
        if((joystick.getButtonB() == false) && joystick.getButtonA()== false)
        {
            flipperoutput = 0.0;
        }
        if(flipperstate == 0 && (joystick.getButtonB() == true) && (getSensorValue() == 0.0)) //move to out position
		{
            flipperoutput = 0.2; //change to actual value later
            flipperstate ++;
		}
        if (flipperstate == 1 && joystick.getButtonA()== false && getSensorValue() == 100)//actual encoder value //stopped at extended
        {
            flipperoutput = 0.0;
            flipperstate ++;
        }
        if (flipperstate == 1 && joystick.getButtonA()==true && getSensorValue() != 0.0) //back to carry position
		{
            flipperoutput = -0.2; // change to actual value later
            flipperstate ++;
        }
        if (flipperstate == 2 && getSensorValue() == 0.0) //stop at carry position
        {
            flipperoutput = 0.0;
        }
        return flipperoutput;
    }
    @Override
    public void displaySensorsValue()
    {
        SmartDashboard.putNumber("BackFlipper Position ticks", getSensorValue());
        SmartDashboard.putNumber("BackFlipper Power", BackFlipper.getMotorOutputPercent());
        SmartDashboard.putBoolean("BackFlipper Reached", reachedTarget());
    }
    @Override
    public void initialize() {
        super.initialize();
        BackFlipper.setInverted(false);
    }
    @Override
    public boolean reachedTarget(){
        return Math.abs(getError()) <= BackFlipperConstants.allowableError;
    }
    @Override
    public double getPosition()
    {
        return getSensorValue() * BackFlipperConstants.TICKS_TO_INCHES;
    }
    
}
