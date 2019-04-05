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
        super(new Path[] {Paths.LHRB1, Paths.LHRB2, Paths.LHRB3});
    }
    enum State
    {
        Drive, Place, Drive2, Drive3, Complete
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
        RobotState state = RobotState.getInstance();
        RobotState.Actions actionsState = RobotState.Actions.getInstance();

        if (oopCount < 25)
            oopCount++;
        else
            outputSetPoint = ManipulatorSetPoint.hatch_mid_back;
        if (s == State.Drive)
        {
            FollowPath(0, false);
            
            if (pathFollower.GetState() == HotPathFollower.State.Complete)
            {
                DoOffset();
                s = State.Place;
            }
            
        }
        if (s == State.Place)
        {
            steeringAssist = true;
            if (actionsState.isVisionDistanceAtTarget() && actionsState.isVisionTurnAtTarget())
            {
                manipulatorScore = true;
                if (state.isSpearsClosed())
                    s = State.Complete;
            }
        }
        if (s == State.Drive2)
        {
            FollowPath(2, true);
            if (pathFollower.GetState() == HotPathFollower.State.Complete)
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