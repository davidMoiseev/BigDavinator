/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//Made By: Nicholas Stankovich 2019
//         Donovan Porter
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {
    double Outspeed = 0.75;
    double Inputspeed = -0.75;
    HotSticks joystick;
    public static final int TALON_INTAKE_BELT = 1;


    WPI_TalonSRX BeltTalon = new WPI_TalonSRX(TALON_INTAKE_BELT);
    
    public Intake(HotSticks joystick) {
    this.joystick = joystick;      
    }
    public void Update()
    {
        if(joystick.getButtonA()) {
            runIntake(Outspeed);
        
        }
        if(joystick.getButtonB()) {
            runIntake(Inputspeed);
        }
        if(!joystick.getButtonA() && !joystick.getButtonB()) {
            allOff();
        }
        writeDashboard();
    }
    public void allOff() {

        BeltTalon.set(ControlMode.PercentOutput, 0.0);
    }
    public void runIntake(double Shootspeed)
    {
        BeltTalon.set(ControlMode.PercentOutput, Shootspeed);  
    }
     // double Tspeed = /*HotSticks.DriverLY()*/ joystick.getRawAxis(1);
      /*ideally if joystick is forward, then intake spits out, if joystick is backward, intake pulls in */

      public void writeDashboard(){
        SmartDashboard.putNumber("Outputspeed", Outspeed);
        SmartDashboard.putNumber("Inputspeed", Inputspeed);
        SmartDashboard.putBoolean("ButtonAValue", joystick.getButtonA());
        SmartDashboard.putBoolean("ButtonBValue", joystick.getButtonB());
      }
     //   SmartDashboard.putBoolean("valueA", valueA);
        //SmartDashboard.putBoolean("valueB", valueB);
        //SmartDashboard.putNumber("speedR", );
        //SmartDashboard.putNumber("JoystickL", ());
        //SmartDashboard.putNumber("JoystickR", ());
}
