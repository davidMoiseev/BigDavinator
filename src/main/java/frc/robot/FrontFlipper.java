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
import com.ctre.phoenix.motorcfontrol.can.WPI_TalonSRX;

/**
 * Add your docs here.
 */

public class FrontFlipper extends MotionMagicActuator {
    private int reached = 0;
    TalonSRX frontFlipper = new TalonSRX(1);
    double output = 0.0;
    
public FrontFlipper(int ID)
{
    super(ID);
}
public void Update(HotController joystick)
	{
        frontFlipper.set(ControlMode.PercentOutput, FrontFlipperOutput(joystick));
    }

public double FrontFlipperOutput(HotController joystick)
    //HotController joystick;
    //this.joystick = new joystick(driver);
    {
        double flipperoutput = 0.0;
        if(joystick.getButtonB() == true) //move to out position
		{
            reached = 180;
            flipperoutput = 0.2; //change to actual value later
		}
		if (joystick.getButtonA()==true) //back to carry position
		{
            
            reached = 0;
            flipperoutput = -0.2; // change to actual value later
        }
        else 
        {
            reached = 0;
            flipperoutput = 0.0;
        }
        return flipperoutput;
    }
    @Override
    public void displaySensorsValue()
    {
    }

    @Override
    public boolean reachedTarget(){
        if (reached == 180)
        {
            return true;
        }
        else if (reached != 180)
        {
            return false;
        }
        return false;
    }
    @Override
    public double getPosition()
    {
        return 0;
    }
    
}
