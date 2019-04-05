package frc.robot.auto.modes;

import org.hotteam67.HotPathFollower;
import org.hotteam67.Path;

import frc.robot.DriveTrain;
import frc.robot.RobotCommandProvider;
import frc.robot.RobotState;

public abstract class AutoModeBase extends RobotCommandProvider
{
    protected final HotPathFollower pathFollower;
    protected double leftOffset = 0;
    protected double rightOffset = 0;
    protected double headingOffset = 0;

    protected AutoModeBase(Path[] paths)
    {
        pathFollower = new HotPathFollower(DriveTrain.ENCODER_TO_REVS, DriveTrain.WHEEL_DIAMETER);
        pathFollower.ConfigAngleP(DriveTrain.ANGLE_PID.P);
        pathFollower.ConfigPosPIDVA(DriveTrain.POS_PIDVA.P, DriveTrain.POS_PIDVA.I, DriveTrain.POS_PIDVA.D,
                DriveTrain.POS_PIDVA.V, DriveTrain.POS_PIDVA.A);
        pathFollower.LoadPaths(paths);
    }

    protected void FollowPath(int path, boolean inverted)
    {
        RobotState state = RobotState.getInstance();
        HotPathFollower.Output drive = pathFollower.FollowNextPoint(path,
                (inverted ? -1 : 1) * (state.getLeftDriveEncoder() - leftOffset),
                (inverted ? -1 : 1) * (state.getRightDriveEncoder() - rightOffset),
                state.getHeading());
        LeftDrive = (inverted ? -1 : 1) * drive.Right;
        RightDrive = (inverted ? -1 : 1) * drive.Left;
        turnDrive = drive.Turn;
    }

    protected void DoOffset()
    {
        RobotState state = RobotState.getInstance();
        leftOffset = state.getLeftDriveEncoder();
        rightOffset = state.getRightDriveEncoder();
        headingOffset = state.getHeading();
    }

    public abstract boolean IsComplete();

    public void setFrontFlipper(int bump)
    {
        frontFlipperCount = bump;
    }

    public void setBackFlipper(int bump)
    {
        backFlipperCount = bump;
    }
}