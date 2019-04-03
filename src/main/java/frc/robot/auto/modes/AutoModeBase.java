package frc.robot.auto.modes;

import org.hotteam67.HotPathFollower;
import org.hotteam67.Path;

import frc.robot.DriveTrain;
import frc.robot.RobotCommandProvider;
import frc.robot.RobotState;

public abstract class AutoModeBase extends RobotCommandProvider
{
    protected final HotPathFollower pathFollower;
    protected AutoModeBase(Path[] paths)
    {
        pathFollower = new HotPathFollower(DriveTrain.ENCODER_TO_REVS, DriveTrain.WHEEL_DIAMETER);
        pathFollower.ConfigAngleP(DriveTrain.ANGLE_PID.P);
        pathFollower.ConfigPosPIDVA(DriveTrain.POS_PIDVA.P, DriveTrain.POS_PIDVA.I, DriveTrain.POS_PIDVA.D, DriveTrain.POS_PIDVA.V, DriveTrain.POS_PIDVA.A);
        pathFollower.LoadPaths(paths);
    }

    protected void FollowPath(int path)
    {
        RobotState state = RobotState.getInstance();
        HotPathFollower.Output drive = pathFollower.FollowNextPoint(path, state.getLeftDriveEncoder(),
                state.getRightDriveEncoder(), state.getHeading());
        LeftDrive = drive.Right;
        RightDrive = drive.Left;
        turnDrive = drive.Turn;
    }

    public abstract boolean IsComplete();
}