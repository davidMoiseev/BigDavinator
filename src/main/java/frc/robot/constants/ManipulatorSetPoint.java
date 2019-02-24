/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;

public enum ManipulatorSetPoint
{
    // 0 16 115
    firstPostion(3, 0, 0), carry_front(3, 0, 10), carry_back(3, 0, -10), hatch_low_front(17, 167, 90), hatch_low_back(
            17, -167, -90), hatch_mid_front(5, 14, 90), hatch_mid_back(5, -14, -90), hatch_high_front(33, 8.5,
                    90), hatch_high_back(33, -8.5, -90), cargo_pickup_front(2, 135, 113), cargo_pickup_back(2, -135,
                            -113), cargo_shuttle_front(3, 29, 107), cargo_shuttle_back(3, -29,
                                    -107), cargo_rocketLow_front(21, 160, 72), cargo_rocketLow_back(21, -160,
                                            -72), cargo_rocketMid_front(9.5, 10, 83), cargo_rocketMid_back(9.5, -10,
                                                    -83), cargo_rocketHigh_front(33, 1.5, 61), cargo_rocketHigh_back(33,
                                                            -1.5, -61), manipulator_safe_minimum(36, 0, 0);

    private ManipulatorSetPoint(double elevatorHeight, double armAngle, double wristAngle)
    {
        this.elevatorHeight = elevatorHeight;
        this.armAngle = armAngle;
        this.wristAngle = wristAngle;
    }

    private final double armAngle;
    private final double wristAngle;
    private final double elevatorHeight;

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
}