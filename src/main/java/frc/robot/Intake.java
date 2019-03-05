/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//Made By: Nicholas Stankovich 2019
//         Donovan Porter
//         TJ Meyer
//         Johnny Blackburn
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.hotteam67.HotController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.IRobotCommandProvider;
import frc.robot.constants.WiringIDs;

public class Intake
{
    double Outspeed = 1;
    double Inputspeed = -1;
    HotController joystick;

    WPI_TalonSRX BeltTalon = new WPI_TalonSRX(WiringIDs.INTAKE);

    public Intake(HotController joystick)
    {
        this.joystick = joystick;
    }

    public void Update(IRobotCommandProvider command)
    {
        if (command.IntakeOut())
        {
            runIntake(Outspeed);

        }
        else if (command.IntakeIn())
        {
            runIntake(Inputspeed);
        }
        else
        {
            allOff();
        }
        writeDashboard();
    }

    public void allOff()
    {

        BeltTalon.set(ControlMode.PercentOutput, 0.0);
    }

    public void runIntake(double Shootspeed)
    {
        BeltTalon.set(ControlMode.PercentOutput, Shootspeed);
    }
    // double Tspeed = /*HotSticks.DriverLY()*/ joystick.getRawAxis(1);
    /*
     * ideally if joystick is forward, then intake spits out, if joystick is
     * backward, intake pulls in
     */

    public void writeDashboard()
    {
        SmartDashboard.putNumber("Outputspeed", Outspeed);
        SmartDashboard.putNumber("Inputspeed", Inputspeed);
        SmartDashboard.putBoolean("Left Bumper", joystick.getButtonLeftBumper());
        SmartDashboard.putBoolean("Right Bumper", joystick.getButtonRightBumper());
    }
    // SmartDashboard.putBoolean("valueA", valueA);
    // SmartDashboard.putBoolean("valueB", valueB);
    // SmartDashboard.putNumber("speedR", );
    // SmartDashboard.putNumber("JoystickL", ());
    // SmartDashboard.putNumber("JoystickR", ());
}
