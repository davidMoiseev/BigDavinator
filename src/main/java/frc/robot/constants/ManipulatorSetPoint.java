/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;

public enum ManipulatorSetPoint {

    firstPostion(3,0,0),
    carry_front(3,0,10),
    carry_back (3,0,-10),
    hatch_low_front(3,0,10),
    hatch_low_back (3,0,10),
    hatch_mid_front(3,90,90),
    hatch_mid_back (3,-90,-90),
    hatch_high_front(3,0,10),
    hatch_high_back (3,0,10),
    cargo_pickup_front(3,0,10),
    cargo_pickup_back (3,0,10),
    cargo_shuttle_front(3,0,10),
    cargo_shuttle_back (3,0,10),
    cargo_rocketLow_front(3,0,10),
    cargo_rocketLow_back (3,0,10),
    cargo_rocketMid_front(3,0,10),
    cargo_rocketMid_back (3,0,10),
    cargo_rocketHigh_front(3,0,10),
    cargo_rocketHigh_back (3,0,10);
    

    private ManipulatorSetPoint(double elevatorHeight, double armAngle, double wristAngle) {
        this.elevatorHeight = elevatorHeight;
        this.armAngle = armAngle;
        this.wristAngle = wristAngle;
    }
    private final double armAngle;
    private final double wristAngle;
    private final double elevatorHeight;

    public double wristAngle(){
        return this.wristAngle;
    }
    
    public double armAngle(){
        return this.armAngle;
    }
    
    public double elevatorHeight(){
        return this.elevatorHeight;
    }
}