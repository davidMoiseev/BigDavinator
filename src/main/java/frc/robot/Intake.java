/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/*public class Intake {

    Joystick joystick;
    public static final int TALON_INTAKE_TOP = 2337;
    public static final int TALON_INTAKE_BOTTOM = 4362;

    WPI_TalonSRX TopTalon = new WPI_TalonSRX(TALON_INTAKE_TOP);
    WPI_TalonSRX BottomTalon = new WPI_TalonSRX(TALON_INTAKE_BOTTOM);

    public void intake(){
        TopTalon.set(ControlMode.PercentOutput, 0.0);
        BottomTalon.set(ControlMode.PercentOutput, 0.0);
    }

    public void allOff() {
        TopTalon.set(ControlMode.PercentOutput, 0.0);
        BottomTalon.set(ControlMode.PercentOutput, 0.0);
    }
    
      public void runIntake(){
        double speed = /*HotSticks.DriverLY() joystick.getRawAxis(1);
        /*ideally if joystick is forward, then intake spits out, if joystick is backward, intake pulls in */
       /*TopTalon.set(ControlMode.PercentOutput, speed);
        BottomTalon.set(ControlMode.PercentOutput, -speed);
    }

}
*/