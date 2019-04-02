package frc.robot.auto.modes;

import org.hotteam67.HotPathFollower;
import org.hotteam67.Path;

import frc.robot.DriveTrain;
import frc.robot.RobotCommandProvider;

public abstract class AutoModeBase extends RobotCommandProvider
{
    protected final HotPathFollower pathFollower;
    public AutoModeBase(Path[] paths)
    {
        pathFollower = new HotPathFollower(DriveTrain.SECOND_ENCODER_TO_REVS, DriveTrain.WHEEL_DIAMETER);
        pathFollower.ConfigAngleP(DriveTrain.ANGLE_PID.P);
        pathFollower.ConfigPosPIDVA(DriveTrain.POS_PIDVA.P, DriveTrain.POS_PIDVA.I, DriveTrain.POS_PIDVA.D, DriveTrain.POS_PIDVA.V, DriveTrain.POS_PIDVA.A);
        pathFollower.LoadPaths(paths);
    }

    public abstract boolean IsComplete();
}