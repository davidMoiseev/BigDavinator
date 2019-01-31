/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.hotteam67.HotLogger;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot
{

    public static final int JOYSTICK_DRIVER = 0;
    public static final int JOYSTICK_LY = 1;
    public static final int JOYSTICK_RX = 4;

    XboxController driver = new XboxController(JOYSTICK_DRIVER);

    DriveTrain driveTrain;

    @Override
    public void robotInit()
    {
        driveTrain = new DriveTrain();
        HotLogger.Setup("leftEncoder", "rightEncoder", "currentYaw", "currentVelocityLeft", "currentVelocityRight",
                "Path Points", "Path Heading", "Heading Error", "Turn Output", "Left Path Position",
                "Left Path Velocity", "Left Path Acceleration", "Left Path X", "Left Path Y",
                "Left Path Calculated Output", "Left Path Heading", "Right Path Position", "Right Path Velocity",
                "Right Path Acceleration", "Right Path X", "Right Path Y", "Right Path Calculated Output",
                "Right Path Heading");
    }

    @Override
    public void autonomousInit()
    {
        driveTrain.zeroSensors();
        driveTrain.zeroTalons();
        profileFinished = false;
    }

    boolean profileFinished = false;

    Elevator elevator = new Elevator();

    @Override
    public void autonomousPeriodic()
    {
        // May have to invert driveturn/drivespeed
        driveTrain.readSensors();
        driveTrain.writeLogs();

        if (!profileFinished)
            profileFinished = driveTrain.FollowPath();
        else
            driveTrain.zeroTalons();
    }

    @Override
    public void robotPeriodic()
    {
    }

    @Override
    public void teleopInit()
    {
        driveTrain.zeroSensors();
        elevator.elevatorINIT();
        elevator.zeroElevator();
    }

    boolean rumble = false;
    @Override
    public void teleopPeriodic()
    {
        driveTrain.arcadeDrive(driver.getX(Hand.kRight), driver.getY(Hand.kLeft));
        driveTrain.readSensors();
        driveTrain.writeLogs();

        /*if (hotstick.ButtonA() == true) {
          elevator.lowerElevator();
        } else if (hotstick.ButtonB() == true) {
          elevator.raiseElevator();
        } else if (hotstick.ButtonY() == true) {
          elevator.motionMagicElevatorTop();
        } else if (hotstick.ButtonX() == true) {
          elevator.motionMagicElevatorLow();
        } else {
          elevator.disableElevator();
        }*/

        if (driver.getAButton() == true) {
          elevator.motionMagicElevatorLow();
        } else if (driver.getBButton() == true) {
          elevator.motionMagicElevatorPrettyLow();
        } else if (driver.getXButton() == true) {
          elevator.motionMagicElevatorMid();
        } else if (driver.getYButton() == true) {
          elevator.motionMagicElevatorTop();
        } else {
          elevator.disableElevator();
        }

        SmartDashboard.putNumber("Position", elevator.getElevatorPosition());
        SmartDashboard.putNumber("Error", elevator.getClosedLoopError());
        SmartDashboard.putBoolean("ButtonA", driver.getAButton());
        SmartDashboard.putBoolean("ButtonB", driver.getBButton());
        SmartDashboard.putBoolean("ButtonY", driver.getYButton());
        SmartDashboard.putBoolean("ButtonX", driver.getXButton());
    }

    @Override
    public void testPeriodic()
    {
    }

    @Override
    public void disabledInit()
    {
        driveTrain.zeroSensors();
        driveTrain.zeroTalons();
    }
}
