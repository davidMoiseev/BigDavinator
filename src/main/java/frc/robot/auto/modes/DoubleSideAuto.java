package frc.robot.auto.modes;

import org.hotteam67.Path;
import org.hotteam67.HotPathFollower;

import frc.robot.RobotState;
import frc.robot.constants.ManipulatorSetPoint;

public class DoubleSideAuto extends AutoModeBase
{
    private final double sideAngle;
    private final double playerStationAngle;
    private final double secondHatchAngle;
    public DoubleSideAuto(double sideAngle, double playerStationAngle, double secondHatchAngle)
    {
        super(new Path[] {});
        this.sideAngle = sideAngle;
        this.playerStationAngle = playerStationAngle;
        this.secondHatchAngle = secondHatchAngle;
    }

    enum State
    {
        DriveToFirst, TurnToFirst, PlaceFirst, DrivetoPickup, Pickup, DriveToSecond, PlaceSecond, Complete, BackupFromFirst, TurnToPickup, TurnToSecond, BackupFromPickup, TurnToSecond1, TurnToPickup1
    }
    State s = State.DriveToFirst;

    int oopCount = 0;
    @Override
    public void Update()
    {
        RobotState state = RobotState.getInstance();
        RobotState.Actions actionsState = RobotState.Actions.getInstance();

        useAutoPipeline = true;
        if (s == State.DriveToFirst)
        {
            /*
            DriveStraight(4.75);
            if (DriveOnTarget(4.75))
            {
                DoOffset();
                s = State.TurnToFirst;
            }

            if (oopCount < 25)
                oopCount++;
            else
                outputSetPoint = ManipulatorSetPoint.hatch_low_front;
                */
            TurnToTarget(10);
            if (TurnOnTarget())
            {
                
            }
        }
        if (s == State.TurnToFirst)
        {
            TurnToTarget(sideAngle);
            if (TurnOnTarget())
            {
                DoOffset();
                s = State.PlaceFirst;
            }
        }
        if (s == State.PlaceFirst)
        {
            isWaiting = true;
            
            if (state.isSpearsClosed())
            {
                DoOffset();
                spearsClosed = true;
                s = State.BackupFromFirst;
            }
        }
        if (s == State.BackupFromFirst)
        {
            DriveStraight(-.25);
            if (DriveOnTarget(-.25))
            {
                DoOffset();
                s = State.TurnToPickup1;
            }
        }
        if (s == State.TurnToPickup1)
        {
            TurnToTarget(playerStationAngle);
            if (TurnOnTarget())
            {
                DoOffset();
                outputSetPoint = ManipulatorSetPoint.hatch_out_back;
                s = State.DrivetoPickup;
            }
        }
        if (s == State.DrivetoPickup)
        {
            DriveStraight(-5.2);

            if (DriveOnTarget(-5.2))
            {
                DoOffset();
                s = State.TurnToPickup;
            }
        }
        if (s == State.TurnToPickup)
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
        if (s == State.BackupFromPickup)
        {
            DriveStraight(1.5);
            if (DriveOnTarget(1.5))
            {
                s = State.TurnToSecond1;
                DoOffset();
            }
        }
        if (s == State.TurnToSecond1)
        {
            TurnToTarget(secondHatchAngle);
            if (TurnOnTarget())
            {
                s = State.DriveToSecond;
                outputSetPoint = ManipulatorSetPoint.hatch_low_front;
                DoOffset();
            }
        }
        if (s == State.DriveToSecond)
        {
            DriveStraight(3);
            if (DriveOnTarget(3))
            {
                s = State.TurnToSecond;
                DoOffset();
            }
        }
        if (s == State.TurnToSecond)
        {
            TurnToTarget(0);
            if (TurnOnTarget())
            {
                s = State.PlaceSecond;
                DoOffset();
            }
        }
        if (s == State.PlaceSecond)
        {
            isWaiting = true;

            if (state.isSpearsClosed())
            {
                DoOffset();
                spearsClosed = true;
                isWaiting = false;
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