/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.hotteam67.HotController;

import frc.robot.constants.ManipulatorSetPoints;
import frc.robot.constants.WiringIDs;

/**
 * Add your docs here.
 */
public class Manipulator {

    private enum InitailizationState {
        UNINITALIZED, CALIBRATING, READY
    }

    private Elevator elevator;
    private Intake intake;
    private IntakePneumatics pneumaticIntake;
    private Wrist wrist;
    private Arm arm;

    private ManipulatorSetPoint frontHatchHigh;
    private ManipulatorSetPoint backHatchHigh;

    private ManipulatorSetPoint frontHatchMiddle;
    private ManipulatorSetPoint backHatchMiddle;

    private ManipulatorSetPoint frontHatchLow;
    private ManipulatorSetPoint backHatchLow;

    private ManipulatorSetPoint frontCargoHigh;
    private ManipulatorSetPoint backCargoHigh;

    private ManipulatorSetPoint frontCargoMiddle;
    private ManipulatorSetPoint backCargoMiddle;

    private ManipulatorSetPoint frontCargoLow;
    private ManipulatorSetPoint backCargoLow;

    private ManipulatorSetPoint frontCargoHold;// Placing Cargo in Cargo Hold
    private ManipulatorSetPoint backCargoHold;// Placing Cargo in Cargo Hold

    private ManipulatorSetPoint frontCargoPickup;
    private ManipulatorSetPoint backCargoPickup;

    private ManipulatorSetPoint frontCarry;// Carrying Cargo
    private ManipulatorSetPoint backCarry;// Carrying Cargo
    private HotController driver;
    private HotController operator;
    private ArmPigeon armPigeon;
    private InitailizationState initailizationState;

    public Manipulator(HotController operator, HotController driver, TalonSRX rightElevator, TalonSRX intake) {
        this.elevator = new Elevator(new TalonSRX(WiringIDs.LEFT_ELEVATOR), rightElevator);
        this.wrist = new Wrist(WiringIDs.WRIST);
        this.arm = new Arm(WiringIDs.SHOULDER);
        this.armPigeon = new ArmPigeon(WiringIDs.PIGEON_ARM);

        this.operator = operator;
        this.driver = driver;

        // frontHatchHigh = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_HEIGHT_HIGH_HATCH,
        //         ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_HIGH_HATCH);
        // frontCargoHigh = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_HEIGHT_HIGH_CARGO,
        //         ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_HIGH_CARGO);
        // backHatchHigh = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_HIGH_HATCH_REVERSE,
        //         ManipulatorSetPoints.WRIST_HEIGHT_HIGH_HATCH);
        // backCargoHigh = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_HIGH_CARGO_REVERSE,
        //         ManipulatorSetPoints.WRIST_HEIGHT_HIGH_CARGO);

        // frontHatchMiddle = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_MIDDLE_HATCH,
        //         ManipulatorSetPoints.WRIST_HEIGHT_MIDDLE_HATCH);
        // frontCargoMiddle = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_MIDDLE_CARGO,
        //         ManipulatorSetPoints.WRIST_HEIGHT_MIDDLE_CARGO);
        // backHatchMiddle = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_MIDDLE_HATCH_REVERSE,
        //         ManipulatorSetPoints.WRIST_HEIGHT_MIDDLE_HATCH);
        // backCargoMiddle = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_MIDDLE_CARGO_REVERSE,
        //         ManipulatorSetPoints.WRIST_HEIGHT_MIDDLE_CARGO);

        // frontHatchLow = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_LOW_HATCH,
        //         ManipulatorSetPoints.WRIST_HEIGHT_LOW_HATCH);
        // frontCargoLow = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_LOW_CARGO,
        //         ManipulatorSetPoints.WRIST_HEIGHT_LOW_CARGO);
        // backHatchLow = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_LOW_HATCH_REVERSE,
        //         ManipulatorSetPoints.WRIST_HEIGHT_LOW_HATCH);
        // backCargoLow = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_LOW_CARGO_REVERSE,
        //         ManipulatorSetPoints.WRIST_HEIGHT_LOW_CARGO);

        // frontCargoHold = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_CARGO_HOLD,
        //         ManipulatorSetPoints.WRIST_HEIGHT_CARGO_HOLD);
        // backCargoHold = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_CARGO_HOLD_REVERSE,
        //         ManipulatorSetPoints.WRIST_HEIGHT_CARGO_HOLD);

        // frontCargoPickup = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PICKUP,
        //         ManipulatorSetPoints.WRIST_HEIGHT_Ground);
        // backCargoPickup = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PICKUP_REVERSE,
        //         ManipulatorSetPoints.WRIST_HEIGHT_Ground);

        frontCarry = new ManipulatorSetPoint(ManipulatorSetPoints.CARRY_WRIST_ANGLE,
                ManipulatorSetPoints.CARRY_ARM_ANGLE,ManipulatorSetPoints.CARRY_ELEVATOR_HEIGHT);
        backCarry = new ManipulatorSetPoint(ManipulatorSetPoints.CARRY_WRIST_ANGLE,
                ManipulatorSetPoints.CARRY_ARM_ANGLE,ManipulatorSetPoints.CARRY_ELEVATOR_HEIGHT);
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

    public void Update() {
        intake.Update();
        pneumaticIntake.Update();

        if (operator.getButtonA()) {
            elevator.setTarget(frontCarry.getElevatorHeight());
            // arm.setTarget(frontCarry.getArmAngle());
            // wrist.setTarget(frontCarry.getWristAngle());

        } else if (operator.getButtonB()) {

        } else if (operator.getButtonX()) {

        } else if (operator.getButtonY()) {

        } else if (operator.getButtonLeftBumper()) {

        } else if (operator.getButtonRightBumper()) {

        } else if (operator.getButtonLeftStick()) {

        } else if (operator.getButtonRightStick()) {

        } else {
            elevator.disable();
            arm.disable();
            wrist.disable();
        }
    }
}
