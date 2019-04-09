package frc.robot.auto.modes;

import org.hotteam67.HotLogger;
import org.hotteam67.HotPathFollower;
import org.hotteam67.Path;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.DriveTrain;
import frc.robot.RobotCommandProvider;
import frc.robot.RobotState;
import frc.robot.auto.sweetturn.SweetTurn;

public abstract class AutoModeBase extends RobotCommandProvider
{
    protected final HotPathFollower pathFollower;
    protected final SweetTurn sweetTurn;
    protected double leftOffset = 0;
    protected double rightOffset = 0;
    protected boolean isWaiting = false;

    protected AutoModeBase(Path[] paths)
    {
        pathFollower = new HotPathFollower(DriveTrain.ENCODER_TO_REVS, DriveTrain.WHEEL_DIAMETER);
        pathFollower.ConfigAngleP(DriveTrain.ANGLE_PID.P);
        pathFollower.ConfigPosPIDVA(DriveTrain.POS_PIDVA.P, DriveTrain.POS_PIDVA.I, DriveTrain.POS_PIDVA.D,
                DriveTrain.POS_PIDVA.V, DriveTrain.POS_PIDVA.A);
        pathFollower.LoadPaths(paths);

        sweetTurn = new SweetTurn();
    }

    protected void FollowPath(int path, boolean inverted)
    {
        RobotState state = RobotState.getInstance();
        HotPathFollower.Output drive = pathFollower.FollowNextPoint(path,
                (inverted ? -1 : 1) * (state.getLeftDriveEncoder() - leftOffset),
                (inverted ? -1 : 1) * (state.getRightDriveEncoder() - rightOffset), state.getHeading());
        LeftDrive = (inverted ? -1 : 1) * drive.Right;
        RightDrive = (inverted ? -1 : 1) * drive.Left;
        turnDrive = drive.Turn;
    }

    protected void DoOffset()
    {
        RobotState state = RobotState.getInstance();
        leftOffset = state.getLeftDriveEncoder();
        rightOffset = state.getRightDriveEncoder();
        interrupted = false;
        isWaiting = false;
        sweetTurn.SweetTurnRestart();
        drivingStraight = false;
        LeftDrive = 0;
        RightDrive = 0;
        turnDrive = 0;
    }

    public abstract boolean IsComplete();

    public boolean IsWaiting()
    {
        return isWaiting;
    }

    protected boolean interrupted = false;

    public void AttemptInterrupt()
    {
        interrupted = true;
    }

    protected boolean drivingStraight = false;
    protected double driveStraightHeading = 0;

    public void TurnToTarget(double target)
    {
        double currentHeading = RobotState.getInstance().getHeading();
        double currentSpeed = RobotState.getInstance().getTurnSpeed();

        SmartDashboard.putNumber("currentHeading", currentHeading);
        SmartDashboard.putNumber("currentSpeed", currentSpeed);

        if (!sweetTurn.TurnComplete())
        {
            double turn = sweetTurn.SweetTurnOutput(target, 5.0, .3, currentHeading, currentSpeed);
            turnDrive = turn;
        }
        else
            turnDrive = 0;

        SmartDashboard.putNumber("AAA AUTO TURN", turnDrive);
        HotLogger.Log("Turn Rate", currentSpeed);
    }

    public boolean TurnOnTarget()
    {
        return sweetTurn.TurnComplete();
    }

    public void DriveStraight(double target)
    {
        if (!drivingStraight)
        {
            driveStraightHeading = RobotState.getInstance().getHeading();
            drivingStraight = true;
        }
        double desiredTurn = (RobotState.getInstance().getHeading() - driveStraightHeading) * .01;

        if (Math.abs(desiredTurn) > .1)
            desiredTurn = .1 * Math.signum(desiredTurn);

        double error = target - GetDist();
        double desiredDrive = error * .5;

        if (Math.abs(desiredDrive) - Math.abs(LeftDrive) > .03)
            desiredDrive = LeftDrive + (.03 * Math.signum(desiredDrive));
        if (Math.abs(desiredDrive) < .12)
            desiredDrive = .12 * Math.signum(desiredDrive);

        LeftDrive = desiredDrive;
        RightDrive = desiredDrive;
        turnDrive = desiredTurn;

        SmartDashboard.putNumber("AAA DRIVE ERROR", error);

        if (DriveOnTarget(target))
        {
            LeftDrive = 0;
            RightDrive = 0;
            turnDrive = 0;
        }
        SmartDashboard.putNumber("AAA DRIVE LEFT", LeftDrive);
        SmartDashboard.putNumber("AAA DRIVE RIGHT", RightDrive);
    }

    public double GetDist()
    {
        return ((RobotState.getInstance().getLeftDriveEncoder() - leftOffset) / DriveTrain.ENCODER_TO_REVS) * (DriveTrain.WHEEL_DIAMETER * Math.PI);
    }

    public boolean DriveOnTarget(double dist)
    {
        return Math.abs(dist - GetDist()) < .05;
    }

    public void setFrontFlipper(int bump)
    {
        frontFlipperCount = bump;
    }

    public void setBackFlipper(int bump)
    {
        backFlipperCount = bump;
    }
}