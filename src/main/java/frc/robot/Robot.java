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

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.WiringIDs;
import org.hotteam67.Interpolation;
import edu.wpi.first.wpilibj.Relay;

public class Robot extends TimedRobot
{
    /**
     * Joysticks
     */
    public static final int JOYSTICK_DRIVER = 0;
    public static final int JOYSTICK_OPERATOR = 1;

    HotController driver = new HotController(JOYSTICK_DRIVER);
    Interpolation driverLstick;
    Interpolation driverRstick;
    Interpolation driverLTrigger;
    Interpolation driverRTrigger;

    Interpolation operatorLstick;
    Interpolation operatorRstick;
    Interpolation operatorLTrigger;
    Interpolation operatorRTrigger;
    // XboxController operator = new XboxController(JOYSTICK_OPERATOR);
    DriveTrain driveTrain;
    Manipulator manipulator;
    Compressor compressor;
    /*
     * TalonSRX eleLeft; TalonSRX eleRight;
     */

    /*
     * TalonSRX shoulder; TalonSRX wrist; TalonSRX intake;
     */

     private boolean forceInitailization;

    @Override
    public void robotInit()
    {
    compressor = new Compressor(0);
    compressor.setClosedLoopControl(true);

        TalonSRX rightElevator = new TalonSRX(WiringIDs.RIGHT_ELEVATOR);
        
        TalonSRX intake = new TalonSRX(WiringIDs.INTAKE);
        HotController driver = new HotController(0);
        HotController operator = new HotController(1);
        driveTrain = new DriveTrain(rightElevator, intake);
        manipulator = new Manipulator(operator, driver, rightElevator, intake, driveTrain);
        manipulator.InitializeTalons();
        manipulator.RestartInitialization();
        forceInitailization = true;

        HotLogger.Setup("leftEncoder", "rightEncoder", "currentYaw", "currentVelocityLeft", "currentVelocityRight",
                "leftStick", "StickLY", HotPathFollower.LoggerValues);

        driver.setDeadBandLY(.3);
        driver.setDeadBandLX(.3);
        driver.setDeadBandRX(.3);
        driver.setDeadBandRY(.3);

        /*
         * eleLeft = new TalonSRX(WiringIDs.LEFT_ELEVATOR); eleRight = new
         * TalonSRX(WiringIDs.RIGHT_ELEVATOR);
         * 
         * 
         * eleRight.follow(eleLeft); eleLeft.setInverted(true);
         */

        /*
         * intake = new TalonSRX(WiringIDs.INTAKE); wrist = new
         * TalonSRX(WiringIDs.WRIST); shoulder = new TalonSRX(WiringIDs.SHOULDER);
         */
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
        manipulator.DisplaySensors();
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
        // rumble(driver);
        // rumble(operator);

        HotLogger.Log("StickLY", -driver.getStickLY());
        driveTrain.Update(driver);

        manipulator.Update();

        // eleLeft.set(ControlMode.PercentOutput, operator.getY(Hand.kLeft) / 2);

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
       //More precise than x^2, change the denominator of constant to calibrate
        
       driverLstick = input -> {
            if(input>0) {
                return (Math.sqrt(Math.pow(10, ((input)*5)-5)));
            }
            else {
                return -1*(Math.sqrt(Math.pow(10, ((input)*-5)-5)));
            }
        };

        driverRstick = input -> {
            if(input>0) {
                return (Math.sqrt(Math.pow(10, ((input)*5)-5)));
            }
            else{
                return -1*(Math.sqrt(Math.pow(10, ((input)*-5)-5)));
            }
        };

        driverLTrigger = input -> { return (Math.sqrt(Math.pow(10, ((input)*5)-5)));};
        driverRTrigger = input -> { return (Math.sqrt(Math.pow(10, ((input)*5)-5)));};

        operatorLstick = input -> {
            if(input>0) {
                return (Math.sqrt(Math.pow(10, ((input)*5)-5)));
            }
            else {
                return -1*(Math.sqrt(Math.pow(10, ((input)*-5)-5)));
            }
        };
        operatorRstick = input -> {
            if(input>0) {
                return (Math.sqrt(Math.pow(10, ((input)*5)-5)));
            }
            else {
                return -1*(Math.sqrt(Math.pow(10, ((input)*-5)-5)));
            }
        };
        
        operatorLTrigger = input -> { return (Math.sqrt(Math.pow(10, ((input)*5)-5)));};
        operatorRTrigger = input -> { return (Math.sqrt(Math.pow(10, ((input)*5)-5)));};

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
         * Clicked for the first time, the robotj is stable so start boot calibrate
         */
        if ((SmartDashboard.getBoolean("RobotReady", false) && !pigeonInitializing) || forceInitailization )
        {
            forceInitailization = false;
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
