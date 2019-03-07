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

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.AutonCommandProvider;
import frc.robot.constants.TeleopCommandProvider;
import frc.robot.constants.WiringIDs;

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
    AutonCommandProvider autonCommandProvider;
    /*
     * TalonSRX eleLeft; TalonSRX eleRight;
     */

    /*
     * TalonSRX shoulder; TalonSRX wrist; TalonSRX intake;
     */

    private boolean forceInitialization;
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
        operator = new HotController(1, true);

        teleopCommandProvider = new TeleopCommandProvider(driver, operator);
        autonCommandProvider = new AutonCommandProvider();

        driveTrain = new DriveTrain(rightElevator, intake);
        manipulator = new Manipulator(operator, driver, rightElevator, intake, driveTrain);
        manipulator.InitializeTalons();
        manipulator.RestartInitialization();
        forceInitialization = true;
        

        HotLogger.Setup("matchNumber", "Has Reset Occured", "Compressor Current", DriveTrain.LoggerTags, HotPathFollower.LoggerValues,
                Manipulator.LoggerTags, Arm.LoggerTags, Elevator.LoggerTags, Wrist.LoggerTags);

        driver.setDeadBandLY(.1);
        driver.setDeadBandLX(.1);
        driver.setDeadBandRX(.1);
        driver.setDeadBandRY(.1);
        driveTrain.initUsbCam();
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
        teleopCommandProvider.Update();

        driveTrain.Update(teleopCommandProvider);
        manipulator.Update(teleopCommandProvider, operator);

        driveTrain.updateUsb(1);

        HotLogger.Log("StickLY", -driver.getStickLY());
        HotLogger.Log("Compressor Current", compressor.getCompressorCurrent());

        driveTrain.readSensors();
        driveTrain.writeLogs();


        driveTrain.readSensors();
        driveTrain.writeLogs();
    }

    @Override
    public void robotPeriodic()
    {
        manipulator.DisplaySensors();
    }

    @Override
    public void teleopInit()
    {
    }

    boolean rumble = false;

    @Override
    public void teleopPeriodic()
    {
        // rumble(driver);
        // rumble(operator);
        /*
        if (DriverStation.getInstance() != null)
        {
            int num = DriverStation.getInstance().getMatchNumber();
            SmartDashboard.putNumber("matchNumber", num);
            HotLogger.Log("matchNumber", num);
        }
        */
        teleopCommandProvider.Update();
        driveTrain.Update(teleopCommandProvider);
        manipulator.Update(teleopCommandProvider, operator);
        HotLogger.Log("Compressor Current", compressor.getCompressorCurrent());
        driveTrain.updateUsb(1);
        HotLogger.Log("StickLY", -driver.getStickLY());
        driveTrain.readSensors();
        driveTrain.writeLogs();

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

        /**
         * Clicked for the first time, the robot is stable so start boot calibrate
         */
        if ((SmartDashboard.getBoolean("RobotReady", false) && !pigeonInitializing) || forceInitialization)
        {
            forceInitialization = false;
            driveTrain.CalibratePigeon();
            manipulator.RestartInitialization();
            pigeonInitializing = true;
        }

        /**
         * Pigeon is done initializing but we have not informed the DashBoard
         */
        else if (pigeonInitializing && driveTrain.PigeonReady() && manipulator.isReady())
        {
            pigeonInitializing = false;
            driveTrain.zeroSensors();
            SmartDashboard.putBoolean("PigeonReady", true);
        }

        manipulator.RunManipulatorInitialization();
    }
}
