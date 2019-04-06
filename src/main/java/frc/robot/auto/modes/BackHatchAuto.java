package frc.robot.auto.modes;

import org.hotteam67.HotLogger;
import org.hotteam67.HotPathFollower;
import org.hotteam67.Path;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        Drive, TurnToRocket, Place, BackupFromRocket, Drive3, Pickup, Complete, Shuffle
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
        HotLogger.Log("Auto State", s.name());
        SmartDashboard.putString("Auto State", s.name());
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
                    || (actionsState.isVisionCanSeeTarget() && pathFollower.getPoints() > 80))
            {
                DoOffset();
                s = State.TurnToRocket;
            }
        }
        if (s == State.TurnToRocket)
        {
            TurnToTarget(-140);
            if (turnDrive == 0 || actionsState.isVisionCanSeeTarget())
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
            }
            if (state.isSpearsClosed())
            {
                steeringAssist = false;
                visionDrive = false;
                DoOffset();
                s = State.Shuffle;
            }
        }
        if (s == State.Shuffle)
        {
            LeftDrive = -.3;
            RightDrive = -.3;
            limitSwitchPlace = true;
            if ((RobotState.getInstance().getLeftDriveEncoder() - leftOffset) < -15)
            {
                manipulatorScore = true;
            }
            if (RobotState.getInstance().isSpearsClosed())
                {
                    s = State.BackupFromRocket;
                    LeftDrive = 0;
                    RightDrive = 0;
                }

        }
        if (s == State.BackupFromRocket)
        {
            DriveStraight(20);
            if (DriveOnTarget(20))
            {
                DoOffset();
                limitSwitchPlace = false;
                s = State.Complete;
            }

        }
        if (s == State.Drive3)
        {
            manipulatorScore = false;

            outputSetPoint = ManipulatorSetPoint.carry_back;
            TurnToTarget(0);
            if (turnDrive == 0)
            {
                DoOffset();
                s = State.Complete;
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