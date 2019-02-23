/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;

public enum ManipulatorSetPoint {
//0 16 115
    firstPostion(3,0,0),
    carry_front(5,0,10),//X
    carry_back (5,0,-10),//X
    hatch_low_front(17,167,90),//A
    hatch_low_back (17,-167,-90),//A
    hatch_mid_front(5,14,90),//B
    hatch_mid_back (5,-14,-90),//B
    hatch_high_front(33,8.5,90),//Y
    hatch_high_back (33,-8.5,-90),//Y
    cargo_pickup_front(.5,111,134),//M3-M6
    cargo_pickup_back (.5,-111,-134),//M3-M6
    cargo_shuttle_front(3,29,107),//RB
    cargo_shuttle_back (3,-29,-107),//RB
    cargo_rocketLow_front(21,160,72),//LB
    cargo_rocketLow_back (21,-160,-72),//LB
    cargo_rocketMid_front(9.5,10,83),//LT
    cargo_rocketMid_back (9.5,-10,-83),//LT
    cargo_rocketHigh_front(33,1.5,61),//RT
    cargo_rocketHigh_back(33,-1.5,-61),//RT
    limit_front_low(4, 90, 90),
    limit_front_high(4, 50, 32),
    limit_back_low(4, -90, -90),
    limit_back_high(4, -50, -32);
    
    //Flip on M1 || M2

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