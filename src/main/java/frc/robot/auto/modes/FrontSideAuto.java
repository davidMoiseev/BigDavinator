package frc.robot.auto.modes;

import org.hotteam67.Path;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.constants.ManipulatorSetPoint;

import org.hotteam67.HotPathFollower;

public class FrontSideAuto extends AutoModeBase
{
    enum State
    {
        DriveToFront, PlaceFront, DriveToStation, Pickup, DriveToSide, PlaceSide, Complete, TurnToSide
    }

    State s = State.DriveToFront;

    public FrontSideAuto(Path stationPath, Path sidePath)
    {
        super(new Path[]
        { stationPath, sidePath });
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

        useAutoPipeline = true;

        if (s == State.DriveToFront)
        {
            outputSetPoint = ManipulatorSetPoint.hatch_low_back;
            DriveStraight(-50);
            if (DriveOnTarget(-50) || interrupted)
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
            if (pathFollower.getPoints() > 15)
            {
                outputSetPoint = ManipulatorSetPoint.hatch_out_front;
            }
            if (pathFollower.GetState() == HotPathFollower.State.Complete
                    || (RobotState.Actions.getInstance().isVisionCanSeeTarget() && pathFollower.getPoints() > 80))
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
            if (pathFollower.getPoints() > 15)
            {
                outputSetPoint = ManipulatorSetPoint.hatch_low_back;
            }
            if (pathFollower.getPoints() > 80)
            {
                DoOffset();
                s = State.TurnToSide;
            }
        }
        if (s == State.TurnToSide)
        {
            TurnToTarget(90);
            if (TurnOnTarget())
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
        SmartDashboard.putString("Auto Mode", s.name());
    }

}