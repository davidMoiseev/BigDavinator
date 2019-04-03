package frc.robot.auto.modes;

import org.hotteam67.HotPathFollower;
import org.hotteam67.Path;
import org.hotteam67.HotPathFollower.State;

import frc.robot.Paths;
import frc.robot.RobotCommandProvider;
import frc.robot.RobotState;

public class DriveStraightAuto extends AutoModeBase
{
    public DriveStraightAuto()
    {
        super(new Path[]
        { Paths.TestPath1 });
    }

    @Override
    public void Update()
    {
        FollowPath(0);
        if (IsComplete())
        {
            LeftDrive = 0;
            RightDrive = 0;
            turnDrive = 0;
        }
    }

    @Override
    public boolean IsComplete()
    {
        return pathFollower.GetState() == State.Complete;
    }
}