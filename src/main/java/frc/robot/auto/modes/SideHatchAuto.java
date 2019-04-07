package frc.robot.auto.modes;

import org.hotteam67.Path;
import org.hotteam67.HotPathFollower;

import frc.robot.RobotState;

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

    @Override
    public void Update()
    {
        RobotState state = RobotState.getInstance();
        RobotState.Actions actionsState = RobotState.Actions.getInstance();

        if (s == State.DriveToFirst)
        {
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
                if (!actionsState.isVisionCanSeeTarget()) s = State.Complete;
                else s = State.PlaceFirst;
            }
        }
        if (s == State.PlaceFirst)
        {
            visionDrive = true;
            steeringAssist = true;
            limitSwitchPlace = true;
            if (actionsState.isVisionDistanceAtTarget() && actionsState.isVisionTurnAtTarget())
            {
                manipulatorScore = true;
            }
            if (state.isSpearsClosed())
            {
                DoOffset();
                visionDrive = false;
                steeringAssist = false;
                s = State.DriveToSecond;
            }
        }
        if (s == State.DriveToSecond)
        {
            FollowPath(0, true);
            if (pathFollower.getPoints() > 30)
            {
                manipulatorScore = false;
                limitSwitchPlace = false;
            }
            if (pathFollower.GetState() == HotPathFollower.State.Complete || (pathFollower.getPoints() > 70 && actionsState.isVisionCanSeeTarget()))
            {
                DoOffset();
                s = State.Pickup;
            }
        }
        if (s == State.Pickup)
        {
            s = State.Complete;
        }
        if (s == State.DriveToSecond)
        {
            FollowPath(1, false);
            if (pathFollower.GetState() == HotPathFollower.State.Complete)
            {
                DoOffset();
                s = State.PlaceSecond;
            }
        }
        if (s == State.PlaceSecond)
        {
            visionDrive = true;
            steeringAssist = true;
            limitSwitchPlace = true;
            if (actionsState.isVisionDistanceAtTarget() && actionsState.isVisionTurnAtTarget())
            {
                manipulatorScore = true;
            }
            if (state.isSpearsClosed())
            {
                DoOffset();
                visionDrive = false;
                steeringAssist = false;
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