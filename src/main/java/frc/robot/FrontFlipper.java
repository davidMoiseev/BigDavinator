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
import frc.robot.constants.FrontFlipperConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */

public class FrontFlipper extends MotionMagicActuator {
    TalonSRX frontFlipper = new TalonSRX(13);
    TalonSRX frontFlipperBack = new TalonSRX(14);
    double output = 0.0;
    
public FrontFlipper(TalonSRX frontFlipper, TalonSRX frontFlipperBack)
{
    super(frontFlipper, frontFlipperBack);
    SRX_PID_0.setSlotIdx(FrontFlipperConstants.slotIdx);
    SRX_PID_0.setPidIdx(FrontFlipperConstants.pidIdx);
    SRX_PID_0.setFeedForward(FrontFlipperConstants.feedForward);
    SRX_PID_0.setProportional(FrontFlipperConstants.proportional);
    SRX_PID_0.setDerivative(FrontFlipperConstants.derivative);
    SRX_PID_0.setIntegral(FrontFlipperConstants.integral);
    SRX_PID_0.setFeedbackDevice(FrontFlipperConstants.feedbackDevice);

}
public void Update(HotController joystick)
	{
        frontFlipper.set(ControlMode.PercentOutput, FrontFlipperOutput(joystick));
        frontFlipperBack.set(ControlMode.PercentOutput, -FrontFlipperOutput(joystick));
    }

public double FrontFlipperOutput(HotController joystick)
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
        SmartDashboard.putNumber("FrontFlipper Position ticks", getSensorValue());
        SmartDashboard.putNumber("FrontFlipper Power", frontFlipper.getMotorOutputPercent());
        SmartDashboard.putBoolean("FrontFlipper Reached", reachedTarget());
    }
    @Override
    public void initialize() {
        super.initialize();
        frontFlipper.setInverted(false);
    }
    @Override
    public boolean reachedTarget(){
        return Math.abs(getError()) <= FrontFlipperConstants.allowableError;
    }
    @Override
    public double getPosition()
    {
        return getSensorValue() * FrontFlipperConstants.TICKS_TO_INCHES;
    }
    
}
