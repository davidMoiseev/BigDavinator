/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class Elevator2 extends MotionMagicActuator {

    PigeonIMU pigeon;

    public Elevator2(int primaryCAN_ID, int secondaryCAN_ID, int pigeonID) {
        super(primaryCAN_ID, secondaryCAN_ID);
        pigeon = new PigeonIMU(pigeonID);
    }

    @Override
    public void getError() {
        
    }
    
    @Override
    public void displaySensorsValue() {
        SmartDashboard.putNumber("ElevatorPosition", GetSensorValue());
    }
}
