package frc.robot.manipulator.routines;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.IManipulatorSetPoint;
import frc.robot.constants.ManipulatorSetPoint;
import frc.robot.constants.ManualManipulatorSetPoint;

public class HatchPlacer extends ManipulatorRoutineBase
{
    private static final double ARM_LENGTH = 21;
    private static final double SCORE_DISTANCE_HIGH = 8;
    private static final double SCORE_DISTANCE_MID = 6.5;
    private static final double SCORE_DISTANCE_LOW = 4;
    static final List<ManipulatorSetPoint> highPlacePoints = new ArrayList<>(
            Arrays.asList(ManipulatorSetPoint.hatch_high_back, ManipulatorSetPoint.hatch_high_front));
    static final List<ManipulatorSetPoint> midPlacePoints = new ArrayList<>(
            Arrays.asList(ManipulatorSetPoint.hatch_mid_front, ManipulatorSetPoint.hatch_mid_back));
    public static final int DRIVE_DIST = 5;

    public enum HatchPlacingState
    {
        Off, Placing, Driving, RetractingArm, RetractingAll, Complete
    }

    HatchPlacingState hatchPlacingState = HatchPlacingState.Off;

    private IManipulatorSetPoint lastOutput = null;
    private double leftEncoderStart = 0;
    private double rightEncoderStart = 0;

    public IManipulatorSetPoint GetPlacingSetPoint(final ManipulatorSetPoint setPoint)
    {
        IManipulatorSetPoint scorePosition = GetScorePosition(setPoint);
        IManipulatorSetPoint output = setPoint;
        if (scorePosition == null)
        {
            lastOutput = output;
            return output;
        }

        if (hatchPlacingState == HatchPlacingState.Off)
        {
            hatchPlacingState = HatchPlacingState.Placing;
        }
        if (hatchPlacingState == HatchPlacingState.Placing)
        {
            if (onTarget(scorePosition) && robotState.isSpearsClosed())
            {
                hatchPlacingState = HatchPlacingState.Driving;
                leftEncoderStart = robotState.getLeftDriveEncoder();
                rightEncoderStart = robotState.getRightDriveEncoder();
            }
            else
            {
                output = scorePosition;
            }
        }
        if (hatchPlacingState == HatchPlacingState.Driving)
        {
            double dist = ((leftEncoderStart - robotState.getLeftDriveEncoder())
                    + (rightEncoderStart - robotState.getRightDriveEncoder())) / 2;
            if ((setPoint.armAngle() > 0 && dist > DRIVE_DIST) || (setPoint.armAngle() < 0 && dist < -DRIVE_DIST))
            {
                hatchPlacingState = HatchPlacingState.Complete;
            }
            else
            {
                output = scorePosition;
            }
        }
        // Retracting the arm first
        if (hatchPlacingState == HatchPlacingState.RetractingArm)
        {
            if (onTarget(setPoint.armAngle(), scorePosition.wristAngle(), scorePosition.elevatorHeight()))
            {
                hatchPlacingState = HatchPlacingState.RetractingAll;
            }
            else
            {
                output = new ManualManipulatorSetPoint(setPoint.armAngle(), scorePosition.wristAngle(),
                        scorePosition.elevatorHeight(), scorePosition.frontFlipper(), scorePosition.backFlipper());
            }
        }
        // Retracting all of it, for high setpoints
        if (hatchPlacingState == HatchPlacingState.RetractingAll)
        {
            output = setPoint;
            if (onTarget(output))
                hatchPlacingState = HatchPlacingState.Complete;
        }
        // Continue holding retract all, but we know we are there
        if (hatchPlacingState == HatchPlacingState.Complete)
        {
            output = setPoint;
        }
        lastOutput = output;
        return output;
    }

    public boolean onTarget()
    {
        return onTarget(lastOutput);
    }

    public HatchPlacingState getState()
    {
        return hatchPlacingState;
    }

    private static IManipulatorSetPoint GetScorePosition(ManipulatorSetPoint setPoint)
    {
        IManipulatorSetPoint output = setPoint;
        if (highPlacePoints.contains(setPoint))
        {
            output = CreatePush(setPoint, true, SCORE_DISTANCE_HIGH);
        }
        else if (midPlacePoints.contains(setPoint))
        {
            output = CreatePush(setPoint, true, SCORE_DISTANCE_MID);
        }
        else if (setPoint == ManipulatorSetPoint.hatch_low_front)
        {
            output = CreateLowFrontSetPoint(setPoint);
        }
        else if (setPoint == ManipulatorSetPoint.hatch_low_back)
        {
            output = CreateLowBackSetPoint(setPoint);
        }
        return output;
    }

    private static IManipulatorSetPoint CreateLowFrontSetPoint(ManipulatorSetPoint setPoint)
    {

        IManipulatorSetPoint push = CreatePush(setPoint, false, SCORE_DISTANCE_LOW);
        double wristAngle = push.wristAngle() + (20 * Math.signum(push.wristAngle()));
        return new ManualManipulatorSetPoint(push.armAngle(), wristAngle, push.elevatorHeight(), push.frontFlipper(),
                push.backFlipper());
    }

    private static IManipulatorSetPoint CreateLowBackSetPoint(ManipulatorSetPoint setPoint)
    {
        return CreatePush(setPoint, false, SCORE_DISTANCE_LOW);
    }

    private static IManipulatorSetPoint CreatePush(ManipulatorSetPoint setPoint, boolean useElevator, double xAddition)
    {
        double armAngle = Math.toRadians(setPoint.armAngle());

        double newArmX = Math.sin(armAngle) * ARM_LENGTH;
        double newArmAngle;

        if (setPoint.armAngle() > 0)
        {
            newArmX += xAddition;
            if (newArmX > ARM_LENGTH)
                newArmX = ARM_LENGTH;

            // Inverse sin on -PI/2 < X < PI/2
            newArmAngle = Math.asin(newArmX / ARM_LENGTH);

            // For two solutions, pick the angle that is closer to the initial setpoint
            if (armAngle > (Math.PI / 2.0))
                newArmAngle = Math.PI - newArmAngle;
        }
        else
        {
            newArmX -= xAddition;
            if (newArmX < -ARM_LENGTH)
                newArmX = -ARM_LENGTH;

            // Inverse sin on -PI/2 < X < PI/2
            newArmAngle = Math.asin(newArmX / ARM_LENGTH);

            // For two solutions, pick the angle that is closer to the initial setpoint
            if (armAngle < -(Math.PI / 2.0))
                newArmAngle = -Math.PI - newArmAngle;
        }

        double newElevatorHeight = setPoint.elevatorHeight();
        if (useElevator)
        {
            newElevatorHeight += ARM_LENGTH * (Math.cos(armAngle) - Math.cos(newArmAngle));
            newElevatorHeight = (newElevatorHeight > 33) ? 33 : newElevatorHeight;
        }

        SmartDashboard.putNumber("New Arm Angle", Math.toDegrees(newArmAngle));
        SmartDashboard.putNumber("New Elevator Height", newElevatorHeight);

        return new ManualManipulatorSetPoint(Math.toDegrees(newArmAngle), setPoint.wristAngle(), newElevatorHeight,
                setPoint.frontFlipper(), setPoint.backFlipper());
    }

    @Override
    public void Reset()
    {
        hatchPlacingState = HatchPlacingState.Off;
        lastOutput = null;
    }

    @Override
    public boolean IsActive()
    {
        return !(hatchPlacingState == HatchPlacingState.Off || hatchPlacingState == HatchPlacingState.Complete);
    }

    @Override
    public IManipulatorSetPoint GetLastOutput()
    {
        return lastOutput;
    }
}