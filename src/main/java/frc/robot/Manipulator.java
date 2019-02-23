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
public class Manipulator {

    private enum InitailizationState {
        UNINITALIZED, CALIBRATING, READY
    }

    private enum ManipulatorState {
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
    private boolean backButtonPrevious = true;
    private boolean commandToBack = false;
    private boolean isLeftTriggerPressed;
    private boolean isRightTriggerPressed;

    public Manipulator(HotController operator, HotController driver, TalonSRX rightElevator, TalonSRX intake,
            DriveTrain drivetrain) {
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

    public void InitializeTalons() {
        elevator.initialize();
        wrist.initialize();
        arm.initialize();
    }

    public void RestartInitialization() {
        initailizationState = InitailizationState.UNINITALIZED;
    }

    public boolean isReady() {
        return initailizationState == InitailizationState.READY;
    }

    public void RunManipulatorInitialization() {

        if (initailizationState == InitailizationState.UNINITALIZED) {
            armPigeon.RestartCalibration();
            initailizationState = InitailizationState.CALIBRATING;
        }
        if (initailizationState == InitailizationState.CALIBRATING) {

            armPigeon.CalibratePigeon();
            if (armPigeon.PigeonReady()) {
                initailizationState = InitailizationState.READY;
                manipulatorState = ManipulatorState.packagePosition;
                arm.setPosition(armPigeon.GetAngle());
                wrist.setPosition(armPigeon.GetAngle() - 134);
                elevator.zeroSensors();

            }
        }

    }

    public void DisplaySensors() {
        elevator.displaySensorsValue();
        arm.displaySensorsValue();
        wrist.displaySensorsValue();
    }

    private void Control(ManipulatorSetPoint targetPosition) {

        if (manipulatorState == ManipulatorState.intializing) {
            elevator.disable();
            arm.disable();
            wrist.disable();
        }

        if (manipulatorState == ManipulatorState.packagePosition) {
            elevator.disable();
            arm.disable();
            wrist.disable();
            if (targetPosition != null) {
                manipulatorState = ManipulatorState.outOfPackagePosition;
            }
        }

        if (manipulatorState == ManipulatorState.outOfPackagePosition) {
            elevator.setTarget(ManipulatorSetPoint.firstPostion);
            if (elevator.reachedTarget()) {
                wrist.setTarget(ManipulatorSetPoint.firstPostion);
                if (wrist.reachedTarget()) {
                    arm.setTarget(ManipulatorSetPoint.firstPostion);
                    if (arm.reachedTarget()) {
                        manipulatorState = ManipulatorState.atTarget;
                    }
                }
            }
        }

        if (manipulatorState == ManipulatorState.atTarget) {
            elevator.setTarget(targetPosition);
            wrist.setTarget(targetPosition);
            arm.setTarget(targetPosition);
        }

        if (manipulatorState == ManipulatorState.transition) {

        }
    }

    public void Update() {
        intake.Update();
        pneumaticIntake.Update();
        ManipulatorSetPoint frontTargetPosition = null;
        ManipulatorSetPoint backTargetPosition = null;

        if (operator.getButtonBack()) {
            //flip code
        } else if (operator.getButtonStart()) {
            //flip code
        } else if (operator.getButtonX()) {
            frontTargetPosition = ManipulatorSetPoint.carry_front;
            backTargetPosition = ManipulatorSetPoint.carry_back;
        } else if (operator.getButtonA()) {
            frontTargetPosition = ManipulatorSetPoint.hatch_low_front;
            backTargetPosition = ManipulatorSetPoint.hatch_low_back;
        } else if (operator.getButtonB()) {
            frontTargetPosition = ManipulatorSetPoint.hatch_mid_front;
            backTargetPosition = ManipulatorSetPoint.hatch_mid_front;
        } else if (operator.getButtonY()) {
            frontTargetPosition = ManipulatorSetPoint.hatch_high_front;
            backTargetPosition = ManipulatorSetPoint.hatch_high_back;
        } else if (operator.getButtonLeftBumper()) {
            frontTargetPosition = ManipulatorSetPoint.cargo_rocketLow_front;
            backTargetPosition = ManipulatorSetPoint.cargo_rocketLow_back;
        } else if (isLeftTriggerPressed == true) {
            frontTargetPosition = ManipulatorSetPoint.cargo_rocketMid_front;
            backTargetPosition = ManipulatorSetPoint.cargo_rocketMid_back;
        } else if (isRightTriggerPressed == true) {
            frontTargetPosition = ManipulatorSetPoint.cargo_rocketHigh_front;
            backTargetPosition = ManipulatorSetPoint.cargo_rocketHigh_back;
        } else if (operator.getButtonRightBumper()) {
            frontTargetPosition = ManipulatorSetPoint.cargo_shuttle_front;
            backTargetPosition = ManipulatorSetPoint.cargo_shuttle_back;
        } else if (operator.getButtonLeftStick()) {
            frontTargetPosition = ManipulatorSetPoint.cargo_pickup_front;
            backTargetPosition = ManipulatorSetPoint.cargo_pickup_back;
        } else if (operator.getButtonRightStick()) {
            frontTargetPosition = ManipulatorSetPoint.cargo_pickup_front;
            backTargetPosition = ManipulatorSetPoint.cargo_pickup_back;
        }

        // if ((operator.getButtonStart() && !startButtonPrevious) ||
        //    (operator.getButtonBack() && !startButtonPrevious)) {
        // commandToBack = !commandToBack;
        // }

        if (frontTargetPosition != null || backTargetPosition != null) {
            if (commandToBack) {
                // Control(backTargetPosition);
            } else {
                Control(frontTargetPosition);
            }
        } else {
            elevator.disable();
            arm.disable();
            wrist.disable();
        }

        startButtonPrevious = operator.getButtonStart();
        backButtonPrevious = operator.getButtonBack();


        if (operator.getLeftTrigger() >= .25 ) {
            isLeftTriggerPressed = true;
        } else {
            isLeftTriggerPressed = false;
        }

        if (operator.getRightTrigger() >= .25) {
            isRightTriggerPressed = true;
        } else {
            isRightTriggerPressed = false;
        }
    }
}
