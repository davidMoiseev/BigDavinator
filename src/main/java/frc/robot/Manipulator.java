/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.hotteam67.HotController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.WiringIDs;
import frc.robot.constants.ManipulatorSetPoint;

/**
 * Add your docs here.
 */
public class Manipulator
{

    private enum InitailizationState
    {
        UNINITALIZED, CALIBRATING, READY
    }

    private enum ManipulatorState
    {
        intializing, packagePosition, outOfPackagePosition, transition, atTarget;
    }

    private Elevator elevator;
    private Intake intake;
    private IntakePneumatics pneumaticIntake;
    private Wrist wrist;
    private Arm arm;

    private HotController driver;
    private HotController operator;
    private ArmPigeon armPigeon;
    private DriveTrain drivetrain;
    private InitailizationState initailizationState;
    private ManipulatorState manipulatorState;
    private boolean startButtonPrevious = true;
    private boolean commandToBack = false;

    public Manipulator(HotController operator, HotController driver, TalonSRX rightElevator, TalonSRX intake,
            DriveTrain drivetrain)
    {
        this.elevator = new Elevator(new TalonSRX(WiringIDs.LEFT_ELEVATOR), rightElevator);
        this.wrist = new Wrist(WiringIDs.WRIST);
        this.arm = new Arm(WiringIDs.SHOULDER);
        this.armPigeon = new ArmPigeon(WiringIDs.PIGEON_ARM);
        this.intake = new Intake(driver);
        this.pneumaticIntake = new IntakePneumatics(driver);

        this.operator = operator;
        this.driver = driver;
        this.drivetrain = drivetrain;

        RestartInitialization();
        manipulatorState = ManipulatorState.intializing;

    }

    public void InitializeTalons()
    {
        elevator.initialize();
        wrist.initialize();
        arm.initialize();
    }

    public void RestartInitialization()
    {
        initailizationState = InitailizationState.UNINITALIZED;
    }

    public boolean isReady()
    {
        return initailizationState == InitailizationState.READY;
    }

    public void RunManipulatorInitialization()
    {

        if (initailizationState == InitailizationState.UNINITALIZED)
        {
            armPigeon.RestartCalibration();
            initailizationState = InitailizationState.CALIBRATING;
        }
        if (initailizationState == InitailizationState.CALIBRATING)
        {

            armPigeon.CalibratePigeon();
            if (armPigeon.PigeonReady())
            {
                initailizationState = InitailizationState.READY;
                manipulatorState = ManipulatorState.packagePosition;
                arm.setPosition(armPigeon.GetAngle());
                wrist.setPosition(armPigeon.GetAngle() - 134);
                elevator.zeroSensors();

            }
        }
    }

    public void DisplaySensors()
    {
        elevator.displaySensorsValue();
        arm.displaySensorsValue();
        wrist.displaySensorsValue();
    }

    private void Control(ManipulatorSetPoint targetPosition)
    {

        if (manipulatorState == ManipulatorState.intializing)
        {
            elevator.disable();
            arm.disable();
            wrist.disable();
        }

        if (manipulatorState == ManipulatorState.packagePosition)
        {
            elevator.disable();
            arm.disable();
            wrist.disable();
            if (targetPosition != null)
            {
                manipulatorState = ManipulatorState.outOfPackagePosition;
            }
        }

        if (manipulatorState == ManipulatorState.outOfPackagePosition)
        {
            elevator.setTarget(ManipulatorSetPoint.firstPostion);
            if (elevator.reachedTarget())
            {
                wrist.setTarget(ManipulatorSetPoint.firstPostion);
                if (wrist.reachedTarget())
                {
                    arm.setTarget(ManipulatorSetPoint.firstPostion);
                    if (arm.reachedTarget())
                    {
                        manipulatorState = ManipulatorState.atTarget;
                    }
                }
            }
        }

        if (manipulatorState == ManipulatorState.atTarget)
        {
            elevator.setTarget(targetPosition);
            wrist.setTarget(targetPosition);
            arm.setTarget(targetPosition);
        }

        if (manipulatorState == ManipulatorState.transition)
        {

        }
    }

    public void Update()
    {
        intake.Update();
        pneumaticIntake.Update();
        ManipulatorSetPoint frontTargetPosition = null;
        ManipulatorSetPoint backTargetPosition = null;

        if (operator.getButtonBack())
        {
            frontTargetPosition = ManipulatorSetPoint.carry_front;
            backTargetPosition = ManipulatorSetPoint.carry_front;
        }
        else if (operator.getButtonB())
        {
            frontTargetPosition = ManipulatorSetPoint.hatch_mid_front;
            backTargetPosition = ManipulatorSetPoint.carry_front;
        }
        else if (operator.getButtonA())
        {
            frontTargetPosition = ManipulatorSetPoint.cargo_pickup_front;
            backTargetPosition = ManipulatorSetPoint.carry_front;
        }
        else if (operator.getButtonX())
        {
            frontTargetPosition = ManipulatorSetPoint.cargo_rocketMid_front;
            backTargetPosition = ManipulatorSetPoint.carry_front;
        }
        else if (operator.getButtonY())
        {
            frontTargetPosition = ManipulatorSetPoint.cargo_rocketLow_front;
            backTargetPosition = ManipulatorSetPoint.cargo_rocketLow_front;
        }
        else if (operator.getButtonLeftBumper())
        {
            frontTargetPosition = ManipulatorSetPoint.hatch_low_front;
            backTargetPosition = ManipulatorSetPoint.hatch_low_front;
        }
        else if (operator.getButtonRightBumper())
        {

        }
        else if (operator.getButtonLeftStick())
        {

        }
        else if (operator.getButtonRightStick())
        {

        }

        // if (operator.getButtonStart() && !startButtonPrevious) {
        // commandToBack = !commandToBack;
        // }

        if (frontTargetPosition != null || backTargetPosition != null)
        {
            if (commandToBack)
            {
                // Control(backTargetPosition);
            }
            else
            {
                Control(frontTargetPosition);
            }
        }
        else
        {
            elevator.disable();
            arm.disable();
            wrist.disable();
        }

        startButtonPrevious = operator.getButtonStart();
    }
}
