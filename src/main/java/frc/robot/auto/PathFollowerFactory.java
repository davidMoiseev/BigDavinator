package frc.robot.auto;

import org.hotteam67.HotPathFollower;
import org.hotteam67.Path;

import frc.robot.DriveTrain;

public class PathFollowerFactory
{
    public static HotPathFollower getPathFollower(Path[] paths)
    {
        HotPathFollower follower = new HotPathFollower(DriveTrain.SECOND_ENCODER_TO_REVS, DriveTrain.WHEEL_DIAMETER);
        follower.ConfigAngleP(DriveTrain.ANGLE_PID.P);
        follower.ConfigPosPIDVA(DriveTrain.POS_PIDVA.P, DriveTrain.POS_PIDVA.I, DriveTrain.POS_PIDVA.D, DriveTrain.POS_PIDVA.V, DriveTrain.POS_PIDVA.A);
        return follower;
    }
}