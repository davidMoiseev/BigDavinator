package frc.robot.auto.modes;

import org.hotteam67.HotPathFollower;
import org.hotteam67.Path;

import frc.robot.Paths;
import frc.robot.RobotState;
import frc.robot.constants.ManipulatorSetPoint;
import frc.robot.manipulator.Manipulator;

public class FrontHatchAuto extends AutoModeBase
{
    public FrontHatchAuto()
    {
        super(new Path[] {Paths.TestPath1});
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
            HotPathFollower.Output out = pathFollower.FollowNextPoint(0, state.getLeftDriveEncoder(),
                    state.getRightDriveEncoder(), state.getHeading());
            LeftDrive = out.Left;
            RightDrive = out.Right;
            outputSetPoint = ManipulatorSetPoint.hatch_low_back;

            if (pathFollower.GetState() == HotPathFollower.State.Complete)
            {
                s = State.Place;
            }
        }
        if (s == State.Place)
        {
            LeftDrive = 0;
            RightDrive = 0;
            manipulatorScore = true;

            if (state.isSpearsClosed())
                s = State.Complete;
        }
    }

}