package frc.robot.auto.modes;

import org.hotteam67.Path;
import org.hotteam67.HotPathFollower;

import frc.robot.RobotState;
import frc.robot.constants.ManipulatorSetPoint;

public class DoubleSideAuto extends AutoModeBase
{
    private final double sideAngle;
    public DoubleSideAuto(double sideAngle, Path pathToStation, Path pathToSecondHatch)
    {
        super(new Path[] {pathToStation, pathToSecondHatch});
        this.sideAngle = sideAngle;
    }

    enum State
    {
        DriveToFirst, TurnToFirst, PlaceFirst, DrivetoPickup, Pickup, DriveToSecond, PlaceSecond, Complete
    }
    State s = State.DriveToFirst;

    int oopCount = 0;
    @Override
    public void Update()
    {
        RobotState state = RobotState.getInstance();
        RobotState.Actions actionsState = RobotState.Actions.getInstance();

        if (s == State.DriveToFirst)
        {
            if (oopCount < 25)
                oopCount++;
            else
                outputSetPoint = ManipulatorSetPoint.hatch_low_back;
            DriveStraight(60);
            if (DriveOnTarget(60))
            {
                DoOffset();
                s = State.TurnToFirst;
            }
        }
        if (s == State.TurnToFirst)
        {
            TurnToTarget(sideAngle);
            if (TurnOnTarget() || interrupted)
            {
                DoOffset();
                s = State.PlaceFirst;
            }
        }
        if (s == State.PlaceFirst)
        {
            isWaiting = true;
            
            if (state.isSpearsClosed())
            {
                DoOffset();
                spearsClosed = true;
                s = State.DrivetoPickup;
            }
        }
        if (s == State.DrivetoPickup)
        {
            FollowPath(0, true);
            if (pathFollower.getPoints() > 30)
            {
                outputSetPoint = ManipulatorSetPoint.hatch_out_front;
            }
            if (pathFollower.GetState() == HotPathFollower.State.Complete || (pathFollower.getPoints() > 70 && actionsState.isVisionCanSeeTarget()) || interrupted)
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
                s = State.DriveToSecond;
            }
        }
        if (s == State.DriveToSecond)
        {
            FollowPath(1, false);
            if (pathFollower.getPoints() > 30)
            {
                outputSetPoint = ManipulatorSetPoint.hatch_low_back;
            }
            if (pathFollower.GetState() == HotPathFollower.State.Complete || interrupted)
            {
                DoOffset();
                s = State.PlaceSecond;
            }
        }
        if (s == State.PlaceSecond)
        {
            isWaiting = true;

            if (state.isSpearsClosed())
            {
                DoOffset();
                spearsClosed = true;
                isWaiting = false;
                s = State.Complete;
            }
        }
    }

    @Override
    public boolean IsComplete()
    {
        return s == State.Complete;
    }
}