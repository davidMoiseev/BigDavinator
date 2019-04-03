/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.hotteam67.HotController;
import org.hotteam67.HotLogger;
import org.hotteam67.HotPathFollower;
import org.hotteam67.Interpolation;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.AutoRunner;
import frc.robot.auto.AutoRunner.Auto;
import frc.robot.constants.WiringIDs;
import frc.robot.manipulator.Arm;
import frc.robot.manipulator.Elevator;
import frc.robot.manipulator.Manipulator;
import frc.robot.manipulator.Wrist;

public class Robot extends TimedRobot
{
    /**
     * Joysticks
     */
    public static final int JOYSTICK_DRIVER = 0;
    public static final int JOYSTICK_OPERATOR = 1;

    HotController driver;
    DriveTrain driveTrain;
    Manipulator manipulator;
    Compressor compressor;
    Solenoid climber;

    HotController operator;

    TeleopCommandProvider teleopCommandProvider;
    AutoRunner autoRunner = new AutoRunner();

    public int state = 0;
    boolean profileFinished = false;

    @Override
    public void robotInit()
    {
        compressor = new Compressor(0);
        compressor.setClosedLoopControl(true);

        TalonSRX rightElevator = new TalonSRX(WiringIDs.RIGHT_ELEVATOR);
        TalonSRX intake = new TalonSRX(WiringIDs.INTAKE);
        this.driver = new HotController(0, false);
        this.operator = new HotController(1, false);

        teleopCommandProvider = new TeleopCommandProvider(driver, operator);

        driveTrain = new DriveTrain(rightElevator, intake);
        manipulator = new Manipulator(rightElevator, intake);
        manipulator.InitializeTalons();
        manipulator.RestartInitialization();

        driver.setDeadBandLY(.1);
        driver.setDeadBandLX(.1);
        driver.setDeadBandRX(.1);
        driver.setDeadBandRY(.1);
        // driveTrain.initUsbCam();
    }

    private void Teleop()
    {
        teleopCommandProvider.Update();
        Run(teleopCommandProvider);
    }

    private void Run(RobotCommandProvider command)
    {
        manipulator.Update(command);
        driveTrain.Update(command);
    }

    @Override
    public void autonomousInit()
    {

        driveTrain.zeroSensors();
        driveTrain.zeroMotors();
        profileFinished = false;
    }

    @Override
    public void autonomousPeriodic()
    {
        // May have to invert driveturn/drivespeed
        if (!autoRunner.IsComplete() && autoRunner.AutoSelected())
        {
            RobotCommandProvider command = autoRunner.Run();
            Run(command);
        }
        else
        {
            Teleop();
        }
    }

    @Override
    public void robotPeriodic()
    {
        manipulator.DisplaySensors();
        driveTrain.readSensors();
        driveTrain.writeLogs();

        manipulator.UpdateRobotState();
        driveTrain.UpdateRobotState();
    }

    @Override
    public void teleopInit()
    {
    }

    boolean rumble = false;

    @Override
    public void teleopPeriodic()
    {
        Teleop();
    }

    /**
     * LETS GET READY TO RUMBBLEEE
     * 
     * @param joy
     */
    public static void rumble(HotController joy)
    {
        double rum = Math.abs(joy.getY(Hand.kLeft)) + Math.abs(joy.getX(Hand.kRight));
        joy.setRumble(RumbleType.kLeftRumble, rum);
        joy.setRumble(RumbleType.kRightRumble, rum);
    }

    @Override
    public void disabledInit()
    {
        // More precise than x^2, change the denominator of constant to calibrate
        driver.setRumble(RumbleType.kLeftRumble, 0);
        driver.setRumble(RumbleType.kRightRumble, 0);
        driveTrain.zeroSensors();
        pigeonInitializing = false;
    }

    /**
     * Whether the dashboard has already told us to configure the pigeon
     */
    boolean pigeonInitializing = false;

    @Override
    public void disabledPeriodic()
    {
        teleopCommandProvider.Reset();
        /**
         * Clicked for the first time, the robot is stable so start boot calibrate
         */

        if ((SmartDashboard.getBoolean("robotReady", false) && !pigeonInitializing))
        {
            SmartDashboard.putBoolean("pigeonReady", false);
            driveTrain.CalibratePigeon();
            manipulator.RestartInitialization();
            NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("stream").setDouble(2);
            NetworkTableInstance.getDefault().getTable("limelight-back").getEntry("stream").setDouble(2);
            pigeonInitializing = true;

            HotLogger.Setup("ENCODER JUMP HAPPENED", "H_DRIVE", "matchNumber", "Has Reset Occured",
                    "Compressor Current", DriveTrain.LoggerTags, HotPathFollower.LoggerValues, Manipulator.LoggerTags,
                    Arm.LoggerTags, Elevator.LoggerTags, Wrist.LoggerTags, TeleopCommandProvider.LoggerTags);

            autoRunner.Select(Auto.BackHatch);
        }

        /**
         * Pigeon is done initializing but we have not informed the DashBoard
         */

        else if (pigeonInitializing && driveTrain.PigeonReady() && manipulator.isReady())
        {
            pigeonInitializing = false;
            driveTrain.zeroSensors();
            SmartDashboard.putBoolean("pigeonReady", true);
            SmartDashboard.putBoolean("robotReady", false);
        }
        //driveTrain.zeroSensors();

        manipulator.RunManipulatorInitialization();
    }
}
