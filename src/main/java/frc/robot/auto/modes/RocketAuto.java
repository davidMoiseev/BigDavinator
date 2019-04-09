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
    private final double rocketAngleFirst;
    private final double rocketAngleSecond;
    private final double angleFromRocket;
    private final double angleToStation;

    public RocketAuto(double rocketAngleFirst, double rocketAngleSecond, double angleFromRocket, double angleToStation)
    {
        super(new Path[]
        { });
        this.rocketAngleFirst = rocketAngleFirst;
        this.rocketAngleSecond = rocketAngleSecond;
        this.angleFromRocket = angleFromRocket;
        this.angleToStation = angleToStation;
    }

    enum State
    {
        DriveOffHab, TurnToRocketFirst, Place, BackupFromRocket, TurnToStation, Pickup, Complete, Shuffle, DriveToStation, DriveToRocket, TurnToRocketSecond, TurnFromRocket, DriveFromRocket
    }

    State s = State.DriveOffHab;

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

        if (s == State.DriveOffHab)
        {   
            DriveStraight(1.5);
            if (DriveOnTarget(1.5))
            {
                DoOffset();
                s = State.TurnToRocketFirst;
            }
        }
        if (s == State.TurnToRocketFirst)
        {
            outputSetPoint = ManipulatorSetPoint.hatch_low_back;
            TurnToTarget(rocketAngleFirst);
            if (TurnOnTarget())
            {
                DoOffset();
                s = State.DriveToRocket;
            }
        }
        if (s == State.DriveToRocket)
        {
            DriveStraight(4.3);
            if (DriveOnTarget(4.3))
            {
                DoOffset();
                s = State.TurnToRocketSecond;
            }
        }
        if (s == State.TurnToRocketSecond)
        {
            TurnToTarget(rocketAngleSecond);
            if (TurnOnTarget())
            {
                DoOffset();
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
            DriveStraight(.25);
            if (DriveOnTarget(.25))
            {
                DoOffset();
                limitSwitchScore = false;
                manipulatorScore = false;
                s = State.TurnFromRocket;
            }

        }
        if (s == State.TurnFromRocket)
        {
            TurnToTarget(angleFromRocket);
            if (TurnOnTarget())
            {
                DoOffset();
                s = State.DriveFromRocket;
            }
        }
        if (s == State.DriveFromRocket)
        {
            DriveStraight(1);
            if (DriveOnTarget(1))
            {
                DoOffset();
                s = State.TurnToStation;
            }
        }

        if (s == State.TurnToStation)
        {
            TurnToTarget(angleToStation);
            if (TurnOnTarget())
            {
                DoOffset();
                s = State.DriveToStation;
            }
            outputSetPoint = ManipulatorSetPoint.hatch_out_back;
        }

        if (s == State.DriveToStation)
        {
            DriveStraight(-4);
            if (DriveOnTarget(-4))
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