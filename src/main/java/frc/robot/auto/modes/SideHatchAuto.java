package frc.robot.auto.modes;

import org.hotteam67.Path;
import org.hotteam67.HotPathFollower;

import frc.robot.RobotState;
import frc.robot.constants.ManipulatorSetPoint;

public class SideHatchAuto extends AutoModeBase
{
    private final double sideAngle;
    public SideHatchAuto(double sideAngle, Path pathToStation, Path pathToSecondHatch)
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
            if (TurnOnTarget())
            {
                DoOffset();
                s = State.PlaceFirst;
            }
        }
        if (s == State.PlaceFirst)
        {
            isWaiting = true;
            // Should be true once auton takes over again
            spearsClosed = true;
            manipulatorScore = true;
            
            if (state.isSpearsClosed())
            {
                DoOffset();
                isWaiting = false;
                s = State.DrivetoPickup;
            }
        }
        if (s == State.DrivetoPickup)
        {
            FollowPath(0, true);
            if (pathFollower.getPoints() > 30)
            {
                manipulatorScore = false;
                limitSwitchScore = false;
                outputSetPoint = ManipulatorSetPoint.hatch_out_front;
            }
            if (pathFollower.GetState() == HotPathFollower.State.Complete || (pathFollower.getPoints() > 70 && actionsState.isVisionCanSeeTarget()))
            {
                DoOffset();
                s = State.Pickup;
            }
        }
        if (s == State.Pickup)
        {
            isWaiting = true;

            // Should be false when auton takes over again
            spearsClosed = false;

            if (!state.isSpearsClosed())
            {
                DoOffset();
                isWaiting = false;
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
            if (pathFollower.GetState() == HotPathFollower.State.Complete)
            {
                DoOffset();
                s = State.PlaceSecond;
            }
        }
        if (s == State.PlaceSecond)
        {
            isWaiting = true;

            // Should be true when auton takes over again
            spearsClosed = true;
            if (!state.isSpearsClosed())
            {
                DoOffset();
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