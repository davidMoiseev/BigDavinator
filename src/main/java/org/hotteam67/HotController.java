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


public class HotController extends Joystick{

    public HotController(int port) {
        super (port);
    }

    private double deadbandLeft_X = 0.1;
    private double deadbandLeft_Y = 0.1;
    private double deadbandRight_X = 0.1;
    private double deadbandRight_Y = 0.1;
    private double deadbandLeftTrigger = 0.0;
    private double deadbandRightTrigger = 0.0;

    public void setDeadBandLX(double band){
        deadbandLeft_X = band;

    }
    public void setDeadBandLY(double band){
        deadbandLeft_Y = band;
    }
    public void setDeadBandRX(double band){
        deadbandRight_X = band;
    }
    public void setDeadBandRY(double band){
        deadbandRight_Y = band;
    }
    public void setDeadBandLT(double band){
        deadbandLeftTrigger = band;
    }
    public void setDeadBandRT(double band){
        deadbandRightTrigger = band;
    }
	public double getStickLX() {
        double value = getRawAxis(0);
        double temp;
		if (value < -1 * deadbandLeft_X) {
            temp = (value + deadbandLeft_X) / (1 - deadbandLeft_X);
        } else if (value > deadbandLeft_X) {
            temp = (value - deadbandLeft_X) / (1 - deadbandLeft_X);
        } else {
            temp = 0.0;
        }
        return temp;
    }
    
	public double getStickLY() {
        double value = getRawAxis(1);
        double temp;
		if (value < -1 * deadbandLeft_Y) {
            temp = (value + deadbandLeft_Y) / (1 - deadbandLeft_Y);
        } else if (value > deadbandLeft_Y) {
            temp = (value - deadbandLeft_Y) / (1 - deadbandLeft_Y);
        } else {
            temp = 0.0;
        }
        return temp;
    }
    
	public double getStickRX() {
        double value = getRawAxis(4);
        double temp;
		if (value < -1 * deadbandRight_X) {
            temp = (value + deadbandRight_X) / (1 - deadbandRight_X);
        } else if (value > deadbandRight_X) {
            temp = (value - deadbandRight_X) / (1 - deadbandRight_X);
        } else {
            temp = 0.0;
        }
        return temp;
    }
    
	public double getStickRY() {
        double value = getRawAxis(5);
        double temp;
		if (value < -1 * deadbandRight_Y) {
            temp = (value + deadbandRight_Y) / (1 - deadbandRight_Y);
        } else if (value > deadbandRight_Y) {
            temp = (value - deadbandRight_Y) / (1 - deadbandRight_Y);
        } else {
            temp = 0.0;
        }
        return temp;
    }
    
	public double getLeftTrigger() {
        double value = getRawAxis(2);
        double temp;
		if (value < -1 * deadbandLeftTrigger) {
            temp = (value + deadbandLeftTrigger) / (1 - deadbandLeftTrigger);
        } else if (value > deadbandLeftTrigger) {
            temp = (value - deadbandLeftTrigger) / (1 - deadbandLeftTrigger);
        } else {
            temp = 0.0;
        }
        return temp;
    }
    

	public double getRightTrigger() {
        double value = getRawAxis(3);
        double temp;
		if (value < -1 * deadbandRightTrigger) {
            temp = (value + deadbandRightTrigger) / (1 - deadbandRightTrigger);
        } else if (value > deadbandRightTrigger) {
            temp = (value - deadbandRightTrigger) / (1 - deadbandRightTrigger);
        } else {
            temp = 0.0;
        }
        return temp;
    }
    
	public boolean getButtonA() {
		boolean value = getRawButton(1);
		return value;
    }
    
	public boolean getButtonB() {
		boolean value = getRawButton(2);
		return value;
    }
    
	public boolean getButtonX() {
		boolean value = getRawButton(3);
		return value;
    }
    
	public boolean getButtonY() {
		boolean value = getRawButton(4);
		return value;
    }

    public boolean getButtonRightBumper() {
		boolean value = getRawButton(6);
		return value;
    }

    public boolean getButtonLeftBumper() {
		boolean value = getRawButton(5);
		return value;
    }

    public boolean getButtonBack() {
		boolean value = getRawButton(7);
		return value;
    }

	public boolean getButtonStart() {
		boolean value = getRawButton(8);
		return value;
    }

	public boolean getButtonLeftStick() {
		boolean value = getRawButton(9);
        return value;
        
    }

    public boolean getButtonRightStick() {
		boolean value = getRawButton(10);
		return value;
    }

    public int getDpad(){
        int value = getPOV();
        return value;
    }


}