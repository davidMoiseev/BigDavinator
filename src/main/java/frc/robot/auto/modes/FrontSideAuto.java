package frc.robot.auto.modes;

import org.hotteam67.Path;

import frc.robot.RobotState;
import frc.robot.constants.ManipulatorSetPoint;

import org.hotteam67.HotPathFollower;

public class FrontSideAuto extends AutoModeBase
{
    enum State
    {
        DriveToFront, PlaceFront, DriveToStation, Pickup, DriveToSide, PlaceSide, Complete
    }
    State s = State.DriveToFront;

    public FrontSideAuto(Path stationPath, Path sidePath)
    {
        super(new Path[]{stationPath, sidePath});
    }

    @Override
    public boolean IsComplete()
    {
        return s == State.Complete;
    }

    int oopCount = 0;
    @Override
    public void Update()
    {
        RobotState state = RobotState.getInstance();

        if (s == State.DriveToFront)
        {
            outputSetPoint = ManipulatorSetPoint.hatch_low_back;
            DriveStraight(-20);
            if (DriveOnTarget(-20) || interrupted)
            {
                DoOffset();
                s = State.PlaceFront;
            }
        }
        if (s == State.PlaceFront)
        {
            isWaiting = true;
            if (state.isSpearsClosed())
            {
                DoOffset();
                spearsClosed = true;
                s = State.DriveToStation;
            }
        }
        if (s == State.DriveToStation)
        {
            FollowPath(0, false);
            if (pathFollower.getPoints() > 5)
            {
                outputSetPoint = ManipulatorSetPoint.hatch_out_front;
            }
            if (pathFollower.GetState() == HotPathFollower.State.Complete || interrupted)
            {
                DoOffset();
                s = State.Pickup;
            }
        }
        if (s == State.Pickup)
        {
            isWaiting = true;
            if (!state.isSpearsClosed())
            {
                DoOffset();
                spearsClosed = false;
                s = State.DriveToSide;
            }
        }
        if (s == State.DriveToSide)
        {
            FollowPath(1, true);
            if (pathFollower.getPoints() > 5)
            {
                outputSetPoint = ManipulatorSetPoint.hatch_low_back;
            }
            if (pathFollower.GetState() == HotPathFollower.State.Complete || interrupted)
            {
                DoOffset();
                s = State.PlaceSide;
            }
        }
        if (s == State.PlaceSide)
        {
            isWaiting = true;
            if (state.isSpearsClosed())
            {
                DoOffset();
                spearsClosed = true;
                s = State.Complete;
            }
        }
    }

}