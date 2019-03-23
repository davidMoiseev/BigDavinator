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

    // hatch low , cargo pickup

    // 0 16 115
    firstPosition(7, 0, 0, FlipperConstants.CARRY_FRONT, FlipperConstants.CARRY_BACK), 
    carry_front(7, -1, 10, FlipperConstants.CARRY_FRONT, FlipperConstants.CARRY_BACK), // X
    carry_back(7, 1, -10, FlipperConstants.CARRY_FRONT, FlipperConstants.CARRY_BACK), // X
    hatch_low_front(15.75, 164, 67, FlipperConstants.HATCH_FRONT, FlipperConstants.CARRY_BACK), // A
    hatch_low_back(16.5, -160, -87, FlipperConstants.CARRY_FRONT, FlipperConstants.HATCH_BACK), // A
    hatch_mid_front(5, 7, 90, FlipperConstants.HATCH_FRONT, FlipperConstants.CARRY_BACK), // B
    hatch_mid_back(5, -7, -90, FlipperConstants.CARRY_FRONT, FlipperConstants.HATCH_BACK), // B
    hatch_high_front(32, 4.5, 90, FlipperConstants.HATCH_FRONT, FlipperConstants.CARRY_BACK), // Y
    hatch_high_back(32, -4.5, -90, FlipperConstants.CARRY_FRONT, FlipperConstants.HATCH_BACK), // Y
    cargo_pickup_front(1, 111, 134, FlipperConstants.CARRY_FRONT, FlipperConstants.CARRY_BACK), // M3-M6
    cargo_pickup_back(1, -105, -134, FlipperConstants.CARRY_FRONT, FlipperConstants.CARRY_BACK), // M3-M6
    cargo_shuttle_front(3, 29, 107, FlipperConstants.CARRY_FRONT, FlipperConstants.CARRY_BACK), // RB
    cargo_shuttle_back(3, -29, -107, FlipperConstants.CARRY_FRONT, FlipperConstants.CARRY_BACK), // RB
    cargo_rocketLow_front(21, 155, 72, FlipperConstants.CARRY_FRONT, FlipperConstants.CARRY_BACK), // LB
    cargo_rocketLow_back(21, -155, -72, FlipperConstants.CARRY_FRONT, FlipperConstants.CARRY_BACK), // LB
    cargo_rocketMid_front(11, 10, 83, FlipperConstants.CARRY_FRONT, FlipperConstants.CARRY_BACK), // LT
    cargo_rocketMid_back(11, -10, -83, FlipperConstants.CARRY_FRONT, FlipperConstants.CARRY_BACK), // LT
    cargo_rocketHigh_front(33, 1.5, 61, FlipperConstants.CARRY_FRONT, FlipperConstants.CARRY_BACK), // RT
    cargo_rocketHigh_back(33, -1.5, -61, FlipperConstants.CARRY_FRONT, FlipperConstants.CARRY_BACK), // RT
    limit_front_low(4, 90, 90, FlipperConstants.CARRY_FRONT, FlipperConstants.CARRY_BACK),
    limit_front_extra_low(4, 130, 90, FlipperConstants.CARRY_FRONT, FlipperConstants.CARRY_BACK),
    limit_front_high(7, 50, 32, FlipperConstants.CARRY_FRONT, FlipperConstants.CARRY_BACK),
    limit_back_low(4, -90, -90, FlipperConstants.CARRY_FRONT, FlipperConstants.CARRY_BACK),
    limit_back_extra_low(4, -130, -90, FlipperConstants.CARRY_FRONT, FlipperConstants.CARRY_BACK),
    limit_back_high(7, -50, -32, FlipperConstants.CARRY_FRONT, FlipperConstants.CARRY_BACK),
    mikes_set_front(0, 90, 0, FlipperConstants.CARRY_FRONT, FlipperConstants.CARRY_BACK),
    mikes_set_back(0, -90, 0, FlipperConstants.CARRY_FRONT, FlipperConstants.CARRY_BACK),
    cargo_station_front(0, 52, 58, FlipperConstants.CARRY_FRONT, FlipperConstants.CARRY_BACK),
    cargo_station_back(0, -52, -58, FlipperConstants.CARRY_FRONT, FlipperConstants.CARRY_BACK),
    // Deploy the climber only at climber_prep, and then operator goes to climber_down
    climb_prep(26.5, -90, -90, FlipperConstants.CARRY_FRONT, FlipperConstants.CARRY_BACK),
    climber_down(-.75, -90, -90, FlipperConstants.CARRY_FRONT, FlipperConstants.CARRY_BACK),
    climber_on(26.5, -144, -30, FlipperConstants.CARRY_FRONT, FlipperConstants.CARRY_BACK);

    // STILL NEED WRIST AND ARM SET FOR CLIMBER

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
