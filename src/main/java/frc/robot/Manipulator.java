/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.constants.WiringIDs;

/**
 * Add your docs here.
 */
public class Manipulator {

    private Elevator elevator;

    //private Intake intake;
    
    private Wrist wrist;

    private Arm arm;

    public Manipulator(TalonSRX rightElevator, TalonSRX intake) {
        this.elevator = new Elevator( new TalonSRX(WiringIDs.LEFT_ELEVATOR), rightElevator);
        this.wrist = new Wrist(WiringIDs.WRIST);
        this.arm = new Arm(WiringIDs.SHOULDER);

    }

    public void InitializeTalons() {
        elevator.initialize();
        wrist.initialize();
        arm.initialize();
    }

    public void IntializeManipulator(){
        
    }


}