package frc.robot.auto.modes;

import org.hotteam67.HotLogger;
import org.hotteam67.HotPathFollower;
import org.hotteam67.Path;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.Paths;
import frc.robot.RobotState;
import frc.robot.constants.ManipulatorSetPoint;
import frc.robot.manipulator.Manipulator;

public class RocketAuto extends AutoModeBase
{
    private final double rocketAngle;

    public RocketAuto(double rocketAngle, Path rocketPath)
    {
        super(new Path[]
        { rocketPath });
        this.rocketAngle = rocketAngle;
    }

    enum State
    {
        Drive, TurnToRocket, Place, BackupFromRocket, TurnToStation, Pickup, Complete, Shuffle, DriveToStation
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
                outputSetPoint = ManipulatorSetPoint.hatch_low_back;

            FollowPath(0, false);

            if (pathFollower.GetState() == HotPathFollower.State.Complete || pathFollower.getPoints() > 110)
            {
                DoOffset();
                s = State.TurnToRocket;
            }
        }
        if (s == State.TurnToRocket)
        {
            TurnToTarget(rocketAngle);
            if (TurnOnTarget())
            {
                DoOffset();
                turnDrive = 0;
                s = State.Place;
            }
        }
        if (s == State.Place)
        {
            isWaiting = true;

            if (state.isSpearsClosed())
            {
                manipulatorScore = true;
                limitSwitchScore = true;
                DoOffset();
                s = State.BackupFromRocket;
            }
        }
        if (s == State.BackupFromRocket)
        {
            DriveStraight(35);
            if (DriveOnTarget(35))
            {
                DoOffset();
                limitSwitchScore = false;
                manipulatorScore = false;
                s = State.TurnToStation;
            }

        }
        if (s == State.TurnToStation)
        {
            outputSetPoint = ManipulatorSetPoint.carry_back;
            TurnToTarget(0);
            if (turnDrive == 0)
            {
                DoOffset();
                s = State.DriveToStation;
            }
        }

        if (s == State.DriveToStation)
        {
            DriveStraight(-100);
            if (DriveOnTarget(-100))
            {
                DoOffset();
                s = State.Complete;
            }
            if (Math.abs(GetDist()) > 50)
                outputSetPoint = ManipulatorSetPoint.hatch_out_back;
        }

        if (s == State.Pickup)
        {
            isWaiting = true;

            if (!state.isSpearsClosed())
            {
                DoOffset();
                s = State.Complete;
            }
        }

        if (s == State.Complete)
        {
            LeftDrive = 0;
            RightDrive = 0;
            turnDrive = 0;
        }

        SmartDashboard.putNumber("AAA LEFTDRIVE2", LeftDrive);
        SmartDashboard.putNumber("AAA RIGHTDRIVE2", RightDrive);
    }

}