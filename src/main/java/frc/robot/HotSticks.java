/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Made By: Aidan Sweeney 2019

package frc.robot;

/**
 * Add your docs here.
 */

import edu.wpi.first.wpilibj.Joystick;

public class HotSticks extends Joystick{

    public HotSticks(int port) {
        super (port);
    }
    private double test;
    //Setting Left stick deadband
    private double LXDB;
    private double LYDB;
    //Setting Right Stick Deadband
    private double RXDB;
    private double RYDB;
    private double LTDB;
    private double RTDB;


    public void SetDeadBandLX(double Band){
        LXDB = Band;
    }
    public void SetDeadBandLY(double Band){
        LYDB = Band;
    }
    public void SetDeadBandRX(double Band){
        RXDB = Band;
    }
    public void SetDeadBandRY(double Band){
        RYDB = Band;
    }
    public void SetDeadBandLT(double Band){
        LTDB = Band;
    }
    public void SetDeadBandRT(double Band){
        RTDB = Band;
    }
	public double StickLX() {
        double value = getRawAxis(0);
        if ((value < -LXDB) || (value >LXDB)){
            return value;
        }else{
            return 0;
        } 
	}
	public double StickLY() {
		double value = getRawAxis(1);
		if ((value < -1 * LYDB) || (value > LYDB)){
            return value;
        }else{
            return 0;
        } 
	}
	public double StickRX() {
		double value = getRawAxis(4);
		if ((value < -RXDB) || (value >RXDB)){
            return value;
        }else{
            return 0;
        } 
	}
	public double StickRY() {
		double value = getRawAxis(5);
		if ((value < -RYDB) || (value >RYDB)){
            return value;
        }else{
            return 0;
        } 
	}
	public double LeftTrigger() {
		double value = getRawAxis(2);
		if ((value < -LTDB) || (value >LTDB)){
            return value;
        }else{
            return 0;
        } 
	}
	public double RightTrigger() {
		double value = getRawAxis(3);
		if ((value < -RTDB) || (value > RTDB)){
            return value;
        }else{
            return 0;
        } 
	}
	public boolean ButtonA() {
		boolean value = getRawButton(1);
		return value;
	}
	public boolean ButtonB() {
		boolean value = getRawButton(2);
		return value;
	}
	public boolean ButtonX() {
		boolean value = getRawButton(3);
		return value;
	}
	public boolean ButtonY() {
		boolean value = getRawButton(4);
		return value;
    }
    public boolean ButtonRightBumper() {
		boolean value = getRawButton(5);
		return value;
    }
    public boolean ButtonLeftBumper() {
		boolean value = getRawButton(6);
		return value;
    }
    public boolean ButtonBack() {
		boolean value = getRawButton(7);
		return value;
    }
	public boolean ButtonStart() {
		boolean value = getRawButton(8);
		return value;
    }
	public boolean ButtonLeftStick() {
		boolean value = getRawButton(9);
		return value;
    }
    public boolean ButtonRightStick() {
		boolean value = getRawButton(10);
		return value;
    }
    public int Dpad(){
        int value = getPOV();
        return value;
    }


}
