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
import frc.robot.auto.modes.AutoModeBase;
import frc.robot.auto.sweetturn.SweetTurn;
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
    int quitAutonTimer = 0;

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

    boolean offHAB2 = false;

    boolean bButtonPrev = false;

    private void Control()
    {
        teleopCommandProvider.Update();
        if (!useHAB2)
            offHAB2 = true;


        if (driver.getButtonB() && !bButtonPrev)
        {
            if (!offHAB2 && useHAB2)
            {
                offHAB2 = true;
            }
            else if (offHAB2)
            {
                quitAutonTimer++;
                if (quitAutonTimer > 1)
                    quitAuton = true;
            }
        }

        bButtonPrev = driver.getButtonB();
        // May have to invert driveturn/drivespeed
        if (offHAB2 && !autoRunner.IsComplete() && autoRunner.AutoSelected() && !quitAuton)
        {
            AutoModeBase autonCommandProvider = autoRunner.Run();

            teleopCommandProvider.SetSpearsClosed(autonCommandProvider.SpearsClosed());
            if (autonCommandProvider.ManipulatorSetPoint() != null)
            {
                teleopCommandProvider.SetUseBack((autonCommandProvider.ManipulatorSetPoint().armAngle() < 0));
            }

            autonCommandProvider.setFrontFlipper(teleopCommandProvider.FrontFlipperBumpCount());
            autonCommandProvider.setBackFlipper(teleopCommandProvider.BackFlipperBumpCount());

            if (teleopCommandProvider.steeringAssistActivated())
                autonCommandProvider.AttemptInterrupt();

            if (autonCommandProvider.IsWaiting())
            {
                teleopCommandProvider.Rumble();
                // Override operator setpoint
                manipulator.Update(teleopCommandProvider, autonCommandProvider.ManipulatorSetPoint());
                driveTrain.Update(teleopCommandProvider);
                autonCommandProvider.SetSpearsClosed(teleopCommandProvider.SpearsClosed());
                // driveTrain.SetBrakeMode(false);
            }
            else
            {
                manipulator.Update(autonCommandProvider);
                driveTrain.Update(autonCommandProvider);
                // driveTrain.SetBrakeMode(true);
            }
        }
        else if (autoRunner.IsComplete() || !autoRunner.AutoSelected())
        {
            quitAuton = true;
            manipulator.Update(teleopCommandProvider);
            driveTrain.Update(teleopCommandProvider);
            // driveTrain.SetBrakeMode(false);
        }
        else
        {
            manipulator.Update(teleopCommandProvider);
            driveTrain.Update(teleopCommandProvider);
        }
    }

    private int autonomousStartTimer = 0;
    @Override
    public void autonomousInit()
    {
        quitAuton = false;
        driveTrain.zeroSensors();
        driveTrain.zeroMotors();
        profileFinished = false;
        quitAutonTimer = 0;
        autonomousStartTimer = 0;
    }

    boolean quitAuton = true;

    @Override
    public void autonomousPeriodic()
    {
        if (autonomousStartTimer < 50)
        {
            autonomousStartTimer++;
            teleopCommandProvider.Rumble();
        }
        Control();
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

    @Override
    public void teleopPeriodic()
    {
        Control();
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
    boolean useHAB2 = false;

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
            // driveTrain.CalibratePigeon();
            manipulator.RestartInitialization();
            NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("stream").setDouble(2);
            NetworkTableInstance.getDefault().getTable("limelight-back").getEntry("stream").setDouble(2);
            pigeonInitializing = true;

            HotLogger.Setup("Auto State", "Turn Rate", "ENCODER JUMP HAPPENED", "H_DRIVE", "matchNumber",
                    "Has Reset Occured", "Compressor Current", DriveTrain.LoggerTags, HotPathFollower.LoggerValues,
                    Manipulator.LoggerTags, Arm.LoggerTags, Elevator.LoggerTags, Wrist.LoggerTags,
                    TeleopCommandProvider.LoggerTags, SweetTurn.LoggerTags);

            int auto = (int) SmartDashboard.getNumber("Auto Selector", 0);
            boolean isLeft = (SmartDashboard.getNumber("Side Selector", 0) == 0) ? true : false;
            AutoRunner.Auto auton = null;
            switch (auto)
            {
            case 0:
                auton = (isLeft) ? Auto.RocketLeft : Auto.RocketRight;
                break;
            case 1:
                auton = (isLeft) ? Auto.RightSideCargo : Auto.LeftSideCargo;
                break;
            case 2:
                auton = (isLeft) ? Auto.LeftFrontCargoOuter : Auto.RightFrontCargoOuter;
                break;
            case 4:
                auton = null;
                break;
            }

            autoRunner.Select(auton);

            quitAuton = true;

            useHAB2 = SmartDashboard.getBoolean("HAB", false);
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
        // driveTrain.zeroSensors();

        manipulator.RunManipulatorInitialization();
    }
}
