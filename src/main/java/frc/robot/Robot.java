/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

    public static final int JOYSTICK_DRIVER = 0;
    public static final int JOYSTICK_LY = 1;
    public static final int JOYSTICK_RX = 4;

    Joystick driver = new Joystick(JOYSTICK_DRIVER);

    DriveTrain driveTrain;

    @Override
    public void robotInit() {
        driveTrain = new DriveTrain();
    }

    @Override
    public void autonomousInit() {
        driveTrain.zeroSensors();
        driveTrain.zeroTalons();
        /*
        HotLog.Setup("leftEncoder", "rightEncoder", "currentYaw", "currentVelocityLeft", "currentVelocityRight",
                "Left Output", "Right Output", "Path Points", "velocity", "acceleration", "position", "x");
        */
        profileFinished = false;
    }

    boolean profileFinished = false;

    @Override
    public void autonomousPeriodic() {
        // May have to invert driveturn/drivespeed
        driveTrain.readSensors();
        driveTrain.writeDashBoard();

        if (!profileFinished)
            profileFinished = driveTrain.FollowPath();
        else
            driveTrain.zeroTalons();

        HotLog.WriteToFile();
    }

    @Override
    public void robotPeriodic() {
    }

    @Override
    public void teleopInit() {
        // HotLog.Setup("leftEncoder", "rightEncoder", "currentYaw",
        // "currentVelocityLeft", "currentVelocityRight");
        driveTrain.zeroSensors();
    }

    @Override
    public void teleopPeriodic() {
        // May have to invert driveturn/drivespeed
        driveTrain.arcadeDrive(driver.getRawAxis(JOYSTICK_RX), driver.getRawAxis(JOYSTICK_LY));
        driveTrain.readSensors();
        driveTrain.writeDashBoard();
        // HotLog.WriteToFile();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void disabledInit() {
    }
}