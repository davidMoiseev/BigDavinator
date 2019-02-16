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

    private Elevator elevator;
    //private Intake intake;
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

    private ManipulatorSetPoint frontCargoHold;//Placing Cargo in Cargo Hold
    private ManipulatorSetPoint backCargoHold;//Placing Cargo in Cargo Hold

    private ManipulatorSetPoint frontCargoPickup;
    private ManipulatorSetPoint backCargoPickup;

    private ManipulatorSetPoint frontCargoCarry;//Carrying Cargo
    private ManipulatorSetPoint backCargoCarry;//Carrying Cargo
    private HotController operator;

    public Manipulator(HotController operator, TalonSRX rightElevator, TalonSRX intake) {
        this.elevator = new Elevator(new TalonSRX(WiringIDs.LEFT_ELEVATOR), rightElevator);
        this.wrist = new Wrist(WiringIDs.WRIST);
        this.arm = new Arm(WiringIDs.SHOULDER);

        this.operator = operator;

        frontHatchHigh = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_HEIGHT_HIGH_HATCH, ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_HIGH_HATCH);
        frontCargoHigh = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_HEIGHT_HIGH_CARGO, ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_HIGH_CARGO);
        backHatchHigh = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_HIGH_HATCH_REVERSE, ManipulatorSetPoints.WRIST_HEIGHT_HIGH_HATCH);
        backCargoHigh = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_HIGH_CARGO_REVERSE, ManipulatorSetPoints.WRIST_HEIGHT_HIGH_CARGO);

        frontHatchMiddle = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_MIDDLE_HATCH, ManipulatorSetPoints.WRIST_HEIGHT_MIDDLE_HATCH);
        frontCargoMiddle = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_MIDDLE_CARGO, ManipulatorSetPoints.WRIST_HEIGHT_MIDDLE_CARGO);
        backHatchMiddle = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_MIDDLE_HATCH_REVERSE, ManipulatorSetPoints.WRIST_HEIGHT_MIDDLE_HATCH);
        backCargoMiddle = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_MIDDLE_CARGO_REVERSE, ManipulatorSetPoints.WRIST_HEIGHT_MIDDLE_CARGO);
    
        frontHatchLow = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_LOW_HATCH, ManipulatorSetPoints.WRIST_HEIGHT_LOW_HATCH);
        frontCargoLow = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_LOW_CARGO, ManipulatorSetPoints.WRIST_HEIGHT_LOW_CARGO);
        backHatchLow = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_LOW_HATCH_REVERSE, ManipulatorSetPoints.WRIST_HEIGHT_LOW_HATCH);
        backCargoLow = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_LOW_CARGO_REVERSE, ManipulatorSetPoints.WRIST_HEIGHT_LOW_CARGO);

        frontCargoHold = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_CARGO_HOLD, ManipulatorSetPoints.WRIST_HEIGHT_CARGO_HOLD);
        backCargoHold = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PLACEMENT_CARGO_HOLD_REVERSE, ManipulatorSetPoints.WRIST_HEIGHT_CARGO_HOLD);
    
        frontCargoPickup = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PICKUP, ManipulatorSetPoints.WRIST_HEIGHT_Ground);
        backCargoPickup = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_PICKUP_REVERSE, ManipulatorSetPoints.WRIST_HEIGHT_Ground);

        frontCargoCarry = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_CARRY, ManipulatorSetPoints.ELEVATOR_HEIGHT_CARRY);
        backCargoCarry = new ManipulatorSetPoint(ManipulatorSetPoints.WRIST_ANGLE_CARRY_REVERSE, ManipulatorSetPoints.ELEVATOR_HEIGHT_CARRY);
    }

    public void InitializeTalons() {
        elevator.initialize();
        wrist.initialize();
        arm.initialize();
    }

    public void IntializeManipulator() {
        elevator.zeroSensors();
        arm.zeroSensors();
        wrist.zeroSensors();
    }         
    
    public void DisplaySensors() {
        elevator.displaySensorsValue();
        arm.displaySensorsValue();
        wrist.displaySensorsValue();
    }

    public void Control() {
        if (operator.getButtonA()) {
            
        } else if(operator.getButtonB()) {
            
        } else if(operator.getButtonX()) {
            
        } else if(operator.getButtonY()) {
            
        } else if(operator.getButtonLeftBumper()) {
            
        } else if(operator.getButtonRightBumper()) {
            
        } else if(operator.getButtonLeftStick()) {
            
        } else if(operator.getButtonRightStick()) {
            
        } else {
            
        }
    }
}