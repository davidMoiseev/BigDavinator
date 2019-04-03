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
        super(new Path[] {Paths.LeftHabBackRocket});
    }
    enum State
    {
        Drive, Place, Complete
    }

    State s = State.Drive;

    @Override
    public boolean IsComplete()
    {
        return s == State.Complete;
    }

    @Override
    public void Update()
    {
        RobotState state = RobotState.getInstance();

        if (s == State.Drive)
        {
            FollowPath(0);
            /*
            outputSetPoint = ManipulatorSetPoint.hatch_low_front;

            if (pathFollower.GetState() == HotPathFollower.State.Complete)
            {
                s = State.Place;
            }
            */
        }
        if (pathFollower.GetState() == HotPathFollower.State.Complete)
        {
            s = State.Complete;
            LeftDrive = 0;
            RightDrive = 0;
            turnDrive = 0;
        }
        /*
        if (s == State.Place)
        {
            LeftDrive = 0;
            RightDrive = 0;
            manipulatorScore = true;

            
            if (state.isSpearsClosed())
            
                s = State.Complete;
        }
        */
    }

}