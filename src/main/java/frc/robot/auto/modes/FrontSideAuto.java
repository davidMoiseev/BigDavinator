package frc.robot.auto.modes;

import org.hotteam67.Path;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.constants.ManipulatorSetPoint;

import org.hotteam67.HotPathFollower;

public class FrontSideAuto extends AutoModeBase
{
    enum State
    {
        DriveToFront, PlaceFront, DriveToStation, Pickup, DriveToSide, PlaceSide, Complete, TurnToSide, BackupFromStation, TurnToStation, TurnToStation2
    }

    State s = State.DriveToFront;
    private final double stationAngle;

    public FrontSideAuto(double stationAngle)
    {
        super(new Path[]
        {});
        this.stationAngle = stationAngle;
    }

    @Override
    public boolean IsComplete()
    {
        return s == State.Complete;
    }

    int oopCount = 0;

    @Override
    public void Update()
    {
        RobotState state = RobotState.getInstance();

        useAutoPipeline = true;

        if (s == State.DriveToFront)
        {
            if (oopCount < 15)
                oopCount++;
            else
                outputSetPoint = ManipulatorSetPoint.hatch_low_front;
            DriveStraight(3);
            if (DriveOnTarget(3))
            {
                DoOffset();
                s = State.PlaceFront;
            }
        }
        if (s == State.PlaceFront)
        {
            isWaiting = true;
            if (state.isSpearsClosed())
            {
                DoOffset();
                spearsClosed = true;
                s = State.BackupFromStation;
            }
        }
        if (s == State.BackupFromStation)
        {
            DriveStraight(-.5);
            if (DriveOnTarget(-.5))
            {
                DoOffset();
                s = State.TurnToStation;
            }
        }
        if (s == State.TurnToStation)
        {
            outputSetPoint = ManipulatorSetPoint.hatch_out_back;
            TurnToTarget(stationAngle);
            if (TurnOnTarget())
            {
                DoOffset();
                s = State.DriveToStation;
            }
        }
        if (s == State.DriveToStation)
        {
            DriveStraight(-4);
            if (DriveOnTarget(-4))
            {
                DoOffset();
                s = State.TurnToStation2;
            }
        }
        if (s == State.TurnToStation2)
        {
            TurnToTarget(0);
            if (TurnOnTarget())
            {
                DoOffset();
                s = State.Pickup;
            }
        }
        if (s == State.Pickup)
        {
            isWaiting = true;
            if (!state.isSpearsClosed())
            {
                DoOffset();
                spearsClosed = false;
                s = State.Complete;
            }
        }
        SmartDashboard.putString("Auto Mode", s.name());
    }

}