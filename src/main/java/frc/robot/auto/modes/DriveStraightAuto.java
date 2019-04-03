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
        RobotState robotState = RobotState.getInstance();

        HotPathFollower.Output drive = pathFollower.FollowNextPoint(0, robotState.getLeftDriveEncoder(),
                robotState.getRightDriveEncoder(), -robotState.getHeading());
        LeftDrive = drive.Right;
        RightDrive = drive.Left;
        turnDrive = -drive.Turn;

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