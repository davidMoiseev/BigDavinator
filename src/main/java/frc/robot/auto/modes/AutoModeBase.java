package frc.robot.auto.modes;

import org.hotteam67.HotPathFollower;
import org.hotteam67.Path;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.DriveTrain;
import frc.robot.RobotCommandProvider;
import frc.robot.RobotState;

public abstract class AutoModeBase extends RobotCommandProvider
{
    protected final HotPathFollower pathFollower;
    protected double leftOffset = 0;
    protected double rightOffset = 0;
    protected double headingOffset = 0;

    protected AutoModeBase(Path[] paths)
    {
        pathFollower = new HotPathFollower(DriveTrain.ENCODER_TO_REVS, DriveTrain.WHEEL_DIAMETER);
        pathFollower.ConfigAngleP(DriveTrain.ANGLE_PID.P);
        pathFollower.ConfigPosPIDVA(DriveTrain.POS_PIDVA.P, DriveTrain.POS_PIDVA.I, DriveTrain.POS_PIDVA.D,
                DriveTrain.POS_PIDVA.V, DriveTrain.POS_PIDVA.A);
        pathFollower.LoadPaths(paths);
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
        headingOffset = state.getHeading();
        drivingStraight = false;
    }

    public abstract boolean IsComplete();

    protected boolean drivingStraight = false;
    protected double driveStraightHeading = 0;

    public void TurnToTarget(double heading)
    {
        double error = (RobotState.getInstance().getHeading() - heading);
        SmartDashboard.putNumber("TURN ERROR", error);
        if (Math.abs(error) > .5)
        {
            turnDrive = error * .003;
            if (Math.abs(turnDrive) < .05)
                turnDrive = .005 * Math.signum(turnDrive);
        }
        else
        {
            turnDrive = 0;
        }
    }

    public void DriveStraight(double dist)
    {
        if (!drivingStraight)
        {
            driveStraightHeading = RobotState.getInstance().getHeading();
            drivingStraight = true;
        }
        double out = (RobotState.getInstance().getHeading() - driveStraightHeading) * .05;
        turnDrive = out;
        double error = dist - (RobotState.getInstance().getLeftDriveEncoder() - leftOffset);
        LeftDrive = error * .05;
        RightDrive = error * .05;

        if (error < .5)
        {
            LeftDrive = 0;
            RightDrive = 0;
            turnDrive = 0;
        }
    }

    public boolean DriveOnTarget(double dist)
    {
        return (RobotState.getInstance().getLeftDriveEncoder() - leftOffset) > dist;
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