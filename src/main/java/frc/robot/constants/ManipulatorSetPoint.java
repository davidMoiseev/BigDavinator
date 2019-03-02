/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;


public enum ManipulatorSetPoint implements IManipulatorSetPoint
{
    // Inches and degrees :)

    // 0 16 115
    firstPosition(7, 0, 0, 0, 0), 
    carry_front(7, 0, 10, 0, 0), // X
    carry_back(7, 0, -10, 0, 0), // X
    hatch_low_front(17, 155, 90, 110, 0), // A
    hatch_low_back(17, -155, -90, 0, 110), // A
    hatch_mid_front(5, 14, 90, 0, 0), // B
    hatch_mid_back(5, -14, -90, 0, 0), // B
    hatch_high_front(32, 8.5, 90, 0, 0), // Y
    hatch_high_back(32, -8.5, -90, 0, 0), // Y
    cargo_pickup_front(.5, 105, 134, 0, 0), // M3-M6
    cargo_pickup_back(.5, -105, -134, 0, 0), // M3-M6
    cargo_shuttle_front(3, 29, 107, 110, 0), // RB
    cargo_shuttle_back(3, -29, -107, 0, 110), // RB
    cargo_rocketLow_front(21, 155, 72, 0, 0), // LB
    cargo_rocketLow_back(21, -155, -72, 0, 0), // LB
    cargo_rocketMid_front(9.5, 10, 83, 0, 0), // LT
    cargo_rocketMid_back(9.5, -10, -83, 0, 0), // LT
    cargo_rocketHigh_front(33, 1.5, 61, 0, 0), // RT
    cargo_rocketHigh_back(33, -1.5, -61, 0, 0), // RT
    climb(3, 0, 0, 0, 0), 
    limit_front_low(4, 90, 90, 0, 0), 
    limit_front_high(7, 50, 32, 0, 0),
    limit_back_low(4, -90, -90, 0, 0), 
    limit_back_high(7, -50, -32, 0, 0),
    mikes_set_front(0, 90, 0, 0, 0),
    mikes_set_back(0, -90, 0, 0, 0);

    // Flip on M1 || M2

    private ManipulatorSetPoint(double elevatorHeight, double armAngle, double wristAngle, double frontFlipperPosition,
            double backFlipperPosition)
    {
        this.elevatorHeight = elevatorHeight;
        this.armAngle = armAngle;
        this.wristAngle = wristAngle;
        this.frontFlipper = frontFlipperPosition;
        this.backFlipper = backFlipperPosition;
    }

    private final double armAngle;
    private final double wristAngle;
    private final double elevatorHeight;
    private final double frontFlipper;
    private final double backFlipper;

    public double wristAngle()
    {
        return this.wristAngle;
    }

    public double armAngle()
    {
        return this.armAngle;
    }

    public double elevatorHeight()
    {
        return this.elevatorHeight;
    }

    public double backFlipper()
    {
        return this.backFlipper;

    }

    public double frontFlipper()
    {
        return this.frontFlipper;
    }
}
