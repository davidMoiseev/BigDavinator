/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;

import org.hotteam67.HotLogger;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.WiringIDs;

public class Robot extends TimedRobot
{
    /**
     * Joysticks
     */
    public static final int JOYSTICK_DRIVER = 0;
    public static final int JOYSTICK_OPERATOR = 1;

    XboxController driver = new XboxController(JOYSTICK_DRIVER);
    XboxController operator = new XboxController(JOYSTICK_OPERATOR);

    DriveTrain driveTrain;

    TalonSRX eleLeft;
    TalonSRX eleRight;

    TalonSRX shoulder;
    TalonSRX wrist;
    TalonSRX intake;

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

        eleLeft = new TalonSRX(WiringIDs.LEFT_ELEVATOR);
        eleRight = new TalonSRX(WiringIDs.RIGHT_ELEVATOR);

        
        eleRight.follow(eleLeft);
        eleLeft.setInverted(true);

        intake = new TalonSRX(WiringIDs.INTAKE);
        wrist = new TalonSRX(WiringIDs.WRIST);
        shoulder = new TalonSRX(WiringIDs.SHOULDER);
    }

    @Override
    public void autonomousInit()
    {
        driveTrain.zeroSensors();
        driveTrain.zeroMotors();
        profileFinished = false;
    }

    boolean profileFinished = false;

    @Override
    public void autonomousPeriodic()
    {
        // May have to invert driveturn/drivespeed
        driveTrain.readSensors();
        driveTrain.writeLogs();

        if (!profileFinished)
            profileFinished = driveTrain.FollowPath();
        else
            driveTrain.zeroMotors();
    }

    @Override
    public void robotPeriodic()
    {
    }

    @Override
    public void teleopInit()
    {
        driveTrain.zeroSensors();
    }

    boolean rumble = false;

    @Override
    public void teleopPeriodic()
    {
        rumble(driver);
        rumble(operator);

        driveTrain.arcadeDrive(driver.getX(Hand.kRight), driver.getY(Hand.kLeft), driver.getX(Hand.kLeft));

        eleLeft.set(ControlMode.PercentOutput, operator.getY(Hand.kLeft) / 2);

        driveTrain.readSensors();
        driveTrain.writeLogs();
    }

    /**
     * WHOS READY TO RUMBBLEEE
     * @param joy
     */
    public static void rumble(XboxController joy)
    {
        double rum = Math.abs(joy.getY(Hand.kLeft)) + Math.abs(joy.getX(Hand.kRight));
        joy.setRumble(RumbleType.kLeftRumble, rum);
        joy.setRumble(RumbleType.kRightRumble, rum);
    }

    @Override
    public void disabledInit()
    {
        driver.setRumble(RumbleType.kLeftRumble, 0);
        driver.setRumble(RumbleType.kRightRumble, 0);
        driveTrain.zeroSensors();
        driveTrain.zeroMotors();
        pigeonInitializing = false;
    }

    /**
     * Whether the dashboard has already told us to configure the pigeon
     */
    boolean pigeonInitializing = false;

    @Override
    public void disabledPeriodic()
    {
        /**
         * Clicked for the first time, the robot is stable so start boot calibrate
         */
        if (SmartDashboard.getBoolean("RobotReady", false) && !pigeonInitializing)
        {
            driveTrain.CalibratePigeon();
            pigeonInitializing = true;
        }
        /**
         * Pigeon is done initializing but we have not informed the DashBoard
         */
        else if (pigeonInitializing && driveTrain.PigeonReady())
        {
            pigeonInitializing = false;
            driveTrain.zeroSensors();
            SmartDashboard.putBoolean("PigeonReady", true);
        }
    }
}