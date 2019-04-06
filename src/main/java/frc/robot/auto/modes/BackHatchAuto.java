package frc.robot.auto.modes;

import org.hotteam67.HotPathFollower;
import org.hotteam67.Path;

import frc.robot.Paths;
import frc.robot.RobotState;
import frc.robot.constants.ManipulatorSetPoint;
import frc.robot.manipulator.Manipulator;

public class BackHatchAuto extends AutoModeBase
{
    public BackHatchAuto()
    {
        super(new Path[]
        { Paths.RHRB1, Paths.RHRB2, Paths.RHRB3 });
    }

    enum State
    {
        Drive, Place, Drive2, Drive3, Pickup, Complete
    }

    State s = State.Drive;

    @Override
    public boolean IsComplete()
    {
        return s == State.Complete;
    }

    private double oopCount = 0;

    @Override
    public void Update()
    {
        useAutoPipeline = true;
        RobotState state = RobotState.getInstance();
        RobotState.Actions actionsState = RobotState.Actions.getInstance();

        if (s == State.Drive)
        {

            if (oopCount < 25)
                oopCount++;
            else
                outputSetPoint = ManipulatorSetPoint.hatch_mid_back;

            FollowPath(0, false);

            if (pathFollower.GetState() == HotPathFollower.State.Complete
                    || (actionsState.isVisionCanSeeTarget() && pathFollower.getPoints() > 70))
            {
                DoOffset();
                s = State.Place;
            }

        }
        if (s == State.Place)
        {
            steeringAssist = true;
            visionDrive = true;
            if (actionsState.isVisionDistanceAtTarget() && actionsState.isVisionTurnAtTarget())
            {
                manipulatorScore = true;
                if (state.isSpearsClosed())
                {
                    steeringAssist = false;
                    visionDrive = false;
                    DoOffset();
                    s = State.Drive2;
                }
            }
        }
        if (s == State.Drive2)
        {
            FollowPath(1, false);
            if (pathFollower.GetState() == HotPathFollower.State.Complete)
            {
                s = State.Drive3;
                DoOffset();
            }
        }
        if (s == State.Drive3)
        {
            manipulatorScore = false;

            outputSetPoint = ManipulatorSetPoint.hatch_out_back;
            FollowPath(2, true);
            if (pathFollower.GetState() == HotPathFollower.State.Complete
                    || (actionsState.isVisionCanSeeTarget() && pathFollower.getPoints() > 70))
            {
                DoOffset();
                s = State.Pickup;
            }
        }
        if (s == State.Pickup)
        {
            visionDrive = true;
            steeringAssist = true;
            if (actionsState.isVisionDistanceAtTarget() && actionsState.isVisionTurnAtTarget())
                s = State.Complete;
        }

        if (s == State.Complete)
        {
            LeftDrive = 0;
            RightDrive = 0;
            turnDrive = 0;
        }
    }

}