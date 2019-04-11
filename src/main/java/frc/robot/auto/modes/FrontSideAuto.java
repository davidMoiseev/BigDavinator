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
        DriveToFrontHeadOn, DriveToFront, PlaceFront, DriveToStation, Pickup, DriveToSide, PlaceSide, Complete, TurnToSide, BackupFromStation, TurnToStation, TurnToStation2, TurnToFront, Angled, TurnToFront2, DriveToRocket, PlaceRocket, TurnToSide2
    }

    State s = State.Angled;
    
    private final double stationAngle;
    private final double cargoAngle;
    private final double sideAngle1;
    private final double sideAngle2;

    public FrontSideAuto(boolean isHeadOn, double cargoAngle, double stationAngle, double sideAngle1, double sideAngle2)
    {
        super(new Path[]
        {});
        this.stationAngle = stationAngle;
        this.cargoAngle = cargoAngle;
        this.sideAngle1 = sideAngle1;
        this.sideAngle2 = sideAngle2;

        if (isHeadOn) s = State.DriveToFrontHeadOn;
    }

    int oopCount = 0;
    @Override
    public boolean IsComplete()
    {
        return s == State.Complete;
    }

    @Override
    public void Update()
    {
        RobotState state = RobotState.getInstance();

        useAutoPipeline = true;

        if (s == State.DriveToFrontHeadOn)
        {
            if (oopCount < 25)
                oopCount++;
            else
                outputSetPoint = ManipulatorSetPoint.hatch_low_front;
            DriveStraight(2);
            if (DriveOnTarget(2))
            {
                DoOffset();
                s = State.PlaceFront;
            }
        }
        if (s == State.Angled)
        {
            DriveStraight(1.5);
            if (DriveOnTarget(1.5))
            {
                DoOffset();
                s = State.TurnToFront;
            }
        }
        if (s == State.TurnToFront)
        {
            outputSetPoint = ManipulatorSetPoint.hatch_low_front;
            TurnToTarget(cargoAngle);
            if (TurnOnTarget())
            {
                DoOffset();
                s = State.DriveToFront;
            }
        }
        if (s == State.DriveToFront)
        {
            DriveStraight(1.5);
            if (DriveOnTarget(1.5))
            {
                DoOffset();
                s = State.TurnToFront2;
            }
        }
        if (s == State.TurnToFront2)
        {
            TurnToTarget(0);
            if (TurnOnTarget())
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
                // s = State.BackupFromStation;
                s = State.Complete;
            }
        }
        if (s == State.BackupFromStation)
        {
            DriveStraight(.25);
            if (DriveOnTarget(.25))
            {
                DoOffset();
                s = State.TurnToSide;
            }
        }
        if (s == State.TurnToSide)
        {
            TurnToTarget(sideAngle1);
            if (TurnOnTarget())
            {
                DoOffset();
                s = State.DriveToSide;
            }
        }
        if (s == State.DriveToSide)
        {
            DriveStraight(4.3);
            if (DriveOnTarget(4.3))
            {
                DoOffset();
                s = State.TurnToSide2;
            }
        }
        if (s == State.TurnToSide2)
        {
            TurnToTarget(sideAngle2);
            if (TurnOnTarget())
            {
                DoOffset();
                s = State.PlaceSide;
            }  
        }
        if (s == State.PlaceSide)
        {
            isWaiting = true;
            if (state.isSpearsClosed())
            {
                DoOffset();
                spearsClosed = true;
                s = State.Complete;
            }
        }
        SmartDashboard.putString("Auto Mode", s.name());
    }

}