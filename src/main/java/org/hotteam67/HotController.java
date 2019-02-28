/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Made By: Nicholas Stankovich 2019
//         Mark Schang
//         Donovan Porter

package org.hotteam67;

/**
 * Add your docs here.
 */

import edu.wpi.first.wpilibj.Joystick;

public class HotController extends Joystick
{
    private final double MAX_COUNT = 10;

    private enum Button
    {
        ButtonA, ButtonB, ButtonX, ButtonY, ButtonRTrigger, ButtonLTrigger, ButtonRStick, ButtonLStick, ButtonLBumper, ButtonRBumper, ButtonBack;
    };

    Button lastButton;

    public HotController(int port, boolean delay)
    {
        super(port);
        this.delay = delay;

    }
    private boolean delay = true;                                                            
    private double deadbandLeft_X = 0.15;
    private double deadbandLeft_Y = 0.15;
    private double deadbandRight_X = 0.15;
    private double deadbandRight_Y = 0.15;
    private double deadbandLeftTrigger = 0.0;
    private double deadbandRightTrigger = 0.0;
    private int counter = 0;

    public void setDeadBandLX(double band)
    {
        deadbandLeft_X = band;

    }

    public void setDeadBandLY(double band)
    {
        deadbandLeft_Y = band;
    }

    public void setDeadBandRX(double band)
    {
        deadbandRight_X = band;
    }

    public void setDeadBandRY(double band)
    {
        deadbandRight_Y = band;
    }

    public void setDeadBandLT(double band)
    {
        deadbandLeftTrigger = band;
    }

    public void setDeadBandRT(double band)
    {
        deadbandRightTrigger = band;
    }

    public double getStickLX()
    {
        double value = getRawAxis(0);
        double temp;
        if (value < -1 * deadbandLeft_X)
        {
            temp = (value + deadbandLeft_X) / (1 - deadbandLeft_X);
        }
        else if (value > deadbandLeft_X)
        {
            temp = (value - deadbandLeft_X) / (1 - deadbandLeft_X);
        }
        else
        {
            temp = 0.0;
        }
        return temp;
    }

    public double getStickLY()
    {
        double value = getRawAxis(1);
        double temp;
        if (value < -1 * deadbandLeft_Y)
        {
            temp = (value + deadbandLeft_Y) / (1 - deadbandLeft_Y);
        }
        else if (value > deadbandLeft_Y)
        {
            temp = (value - deadbandLeft_Y) / (1 - deadbandLeft_Y);
        }
        else
        {
            temp = 0.0;
        }
        return temp;
    }

    public double getStickRX()
    {
        double value = getRawAxis(4);
        double temp;
        if (value < -1 * deadbandRight_X)
        {
            temp = (value + deadbandRight_X) / (1 - deadbandRight_X);
        }
        else if (value > deadbandRight_X)
        {
            temp = (value - deadbandRight_X) / (1 - deadbandRight_X);
        }
        else
        {
            temp = 0.0;
        }
        return temp;
    }

    public double getStickRY()
    {
        double value = getRawAxis(5);
        double temp;
        if (value < -1 * deadbandRight_Y)
        {
            temp = (value + deadbandRight_Y) / (1 - deadbandRight_Y);
        }
        else if (value > deadbandRight_Y)
        {
            temp = (value - deadbandRight_Y) / (1 - deadbandRight_Y);
        }
        else
        {
            temp = 0.0;
        }
        return temp;
    }

    public boolean getLeftTrigger()
    {
        if(getLeftTriggerAnalog() > 0.5)
            {
                lastButton = Button.ButtonLTrigger;
                counter = 0;
                return true;
            }
            if (lastButton == Button.ButtonLTrigger && counter < MAX_COUNT)
            {
                counter++;
                return false; 
            }
            else
            {
            return false;
            }
        }

    public double getLeftTriggerAnalog()
    {
        double value = getRawAxis(2);
        double temp;
        if (value < -1 * deadbandLeftTrigger)
        {
            temp = (value + deadbandLeftTrigger) / (1 - deadbandLeftTrigger);
        }
        else if (value > deadbandLeftTrigger)
        {
            temp = (value - deadbandLeftTrigger) / (1 - deadbandLeftTrigger);
        }
        else
        {
            temp = 0.0;
        }
        return temp;
    }

    public boolean getRightTrigger()
    {
        
        if(getRightTriggerAnalog() > 0.5)
            {
                lastButton = Button.ButtonRTrigger;
                counter = 0;
                return true;
            }
            if (lastButton == Button.ButtonRTrigger && counter < MAX_COUNT)
            {
                counter++;
                return false; 
            }
            else
            {
            return false;
            }
        }

    public double getRightTriggerAnalog()
    {
    double value = getRawAxis(3);
    double temp = 0.0;
        if (value < -1 * deadbandRightTrigger)
        {
            temp = (value + deadbandRightTrigger) / (1 - deadbandRightTrigger);
        }
        else if (value > deadbandRightTrigger)
        {
            temp = (value - deadbandRightTrigger) / (1 - deadbandRightTrigger);
        }
        else
        {
            temp = 0.0;
        }
        return temp;
    }
    public boolean getButtonA()
    {
        lastButton = Button.ButtonA;
    
        if(delay)
        {
            if(getRawButton(1))
            {
                lastButton = Button.ButtonA;
                counter = 0;
                return true;
            }
            else if (lastButton == Button.ButtonA && counter < MAX_COUNT)
            {
                counter++;
                return true; 
            }
            else
            {
                return false;
            }
        }
        else 
        {
            boolean value = getRawButton(1);
            return value;
        }
    }

    public boolean getButtonB()
    {
        lastButton = Button.ButtonB;
        if(delay)
        {
            if(getRawButton(2))
            {
                lastButton = Button.ButtonB;
                counter = 0;
                return true;
            }
            else if (lastButton == Button.ButtonB && counter < MAX_COUNT)
            {
                counter++;
                return true; 
            }
            else
            {
                return false;
            }
        }
        else 
        {
            boolean value = getRawButton(2);
            return value;
        }
    }
    public boolean getButtonX()
    {
        lastButton = Button.ButtonX;
        if(delay)
        {
            if(getRawButton(3))
            {
                lastButton = Button.ButtonX;
                counter = 0;
                return true;
            }
            else if (lastButton == Button.ButtonX && counter < MAX_COUNT)
            {
                counter++;
                return true; 
            }
            else
            {
                return false;
            }
        }
        else 
        {
            boolean value = getRawButton(3);
            return value;
        }
    }
    public boolean getButtonY()
    {
        lastButton = Button.ButtonY;
        if(delay)
        {
            if(getRawButton(4))
            {
                lastButton = Button.ButtonY;
                counter = 0;
                return true;
            }
            else if (lastButton == Button.ButtonY && counter < MAX_COUNT)
            {
                counter++;
                return true; 
            }
            else
            {
                return false;
            }
        }
        else 
        {
            boolean value = getRawButton(4);
            return value;
        }
    }
    public boolean getButtonRightBumper()
   {
        lastButton = Button.ButtonRBumper;
         
        if(delay)
        {
            if(getRawButton(6))
            {
                lastButton = Button.ButtonRBumper;
                counter = 0;
                return true;
            }
            else if (lastButton == Button.ButtonRBumper && counter < MAX_COUNT)
            {
                counter++;
                return true; 
            }
            else
            {
                return false;
            }
        }
        else 
        {
            boolean value = getRawButton(6);
            return value;
        }
    }
    public boolean getButtonLeftBumper()
    {
        lastButton = Button.ButtonLBumper;
          
        if(delay)
        {
            if(getRawButton(5))
            {
                lastButton = Button.ButtonLBumper;
                counter = 0;
                return true;
            }
            else if (lastButton == Button.ButtonLBumper && counter < MAX_COUNT)
            {
                counter++;
                return true; 
            }
            else
            {
                return false;
            }
        }
        else 
        {
            boolean value = getRawButton(5);
            return value;
        }
    }
    public boolean getButtonBack()
    {
        lastButton = Button.ButtonBack;
        //
        if(delay)
        {
            if(getRawButton(7))
            {
                lastButton = Button.ButtonBack;
                counter = 0;
                return true;
            }
            else if (lastButton == Button.ButtonBack && counter < MAX_COUNT)
            {
                counter++;
                return true; 
            }
            else
            {
                return false;
            }
        }
        else 
        {
            boolean value = getRawButton(7);
            return value;
        }
    }
    public boolean getButtonStart()
    {
        boolean value = getRawButton(8);
        return value;
    }

    public boolean getButtonLeftStick()
    {
        lastButton = Button.ButtonLStick;
        //
        if(delay)
        {
            if(getRawButton(9))
            {
                lastButton = Button.ButtonLStick;
                counter = 0;
                return true;
            }
            else if (lastButton == Button.ButtonLStick && counter < MAX_COUNT)
            {
                counter++;
                return true; 
            }
            else
            {
                return false;
            }
        }
        else 
        {
            boolean value = getRawButton(9);
            return value;
        }
    }
    public boolean getButtonRightStick()
    {
        lastButton = Button.ButtonRStick;
        //
        if(delay)
        {
            if(getRawButton(10))
            {
                lastButton = Button.ButtonRStick;
                counter = 0;
                return true;
            }
            else if (lastButton == Button.ButtonRStick && counter < MAX_COUNT)
            {
                counter++;
                return true; 
            }
            else
            {
                return false;
            }
        }
        else 
        {
            boolean value = getRawButton(10);
            return value;
        }
    }
    public int getDpad()
    {
        int value = getPOV();
        return value;
    }

}