package frc.robot.auto.modes;

import org.hotteam67.HotPathFollower;
import org.hotteam67.Path;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.DriveTrain;
import frc.robot.RobotCommandProvider;
import frc.robot.RobotState;
import frc.robot.SweetTurn;

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
        sweetTurn.SweetTurnReset();
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

        if (!sweetTurn.IsTurnComplete())
        {
            double turn = sweetTurn.SweetTurnOutput(target, 5.0, .5, currentHeading, currentSpeed);
            turnDrive = turn;
        }
        else    
            turnDrive = 0;
    }

    public boolean TurnOnTarget()
    {
        return sweetTurn.IsTurnComplete();
    }

    public void DriveStraight(double dist)
    {
        if (!drivingStraight)
        {
            driveStraightHeading = RobotState.getInstance().getHeading();
            drivingStraight = true;
        }
        double out = (RobotState.getInstance().getHeading() - driveStraightHeading) * .05;
        if (Math.abs(out) > .1) out = .1 * Math.signum(out);
        turnDrive = out;
        double error = dist - (RobotState.getInstance().getLeftDriveEncoder() - leftOffset);
        double desiredOut = error * .05;

        if (Math.abs(desiredOut) - Math.abs(LeftDrive) > .025)
            desiredOut = LeftDrive + (.025 * Math.signum(desiredOut));

        LeftDrive = desiredOut;
        RightDrive = desiredOut;

        SmartDashboard.putNumber("AAA DRIVE ERROR", error);

        if (Math.abs(error) < .5)
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
        return (RobotState.getInstance().getLeftDriveEncoder() - leftOffset);
    }

    public boolean DriveOnTarget(double dist)
    {
        return Math.abs(GetDist()) > Math.abs(dist);
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