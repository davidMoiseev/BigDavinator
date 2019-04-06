package frc.robot.manipulator.routines;

import frc.robot.RobotState;
import frc.robot.constants.IManipulatorSetPoint;
import frc.robot.constants.ManipulatorSetPoint;
import frc.robot.constants.ManualManipulatorSetPoint;

public class HatchGrabber extends ManipulatorRoutineBase
{
    /**
     *
     */

    private static final int ARM_PUSH = 11;
    public static final double ELEV_LIFT = 3;
    public static final double DRIVE_DIST = 7;
    public static final double MIN_RELEASE_TIME = 0;

    private double releaseTimer = 0;

    public enum HatchGrabberState
    {
        Off, Pushing, Grabbing, Driving, Complete
    }

    private HatchGrabberState hatchGrabberState = HatchGrabberState.Off;
    private double startEncoderLeft = 0;
    private double startEncoderRight = 0;
    private IManipulatorSetPoint lastOutput;

    public IManipulatorSetPoint GetPickupSetPoint(ManipulatorSetPoint setPoint)
    {
        IManipulatorSetPoint output = setPoint;

        if (hatchGrabberState == HatchGrabberState.Off && robotState.isSpearsClosed())
        {
            if (setPoint == ManipulatorSetPoint.hatch_out_back || setPoint == ManipulatorSetPoint.hatch_out_front)
                hatchGrabberState = HatchGrabberState.Grabbing;
            else
                hatchGrabberState = HatchGrabberState.Pushing;
        }
        if (hatchGrabberState == HatchGrabberState.Pushing)
        {
            output = getPushSetPoint(setPoint);
            if (onTarget(output) && releaseTimer > MIN_RELEASE_TIME)
            {
                hatchGrabberState = HatchGrabberState.Grabbing;
            }
            else if (onTarget(output))
            {
                releaseTimer++;
            }
        }
        if (hatchGrabberState == HatchGrabberState.Grabbing)
        {
            output = getGrabSetPoint(setPoint);
            if (onTarget(output))
            {
                hatchGrabberState = HatchGrabberState.Driving;
                startEncoderLeft = robotState.getLeftDriveEncoder();
                startEncoderRight = robotState.getRightDriveEncoder();
            }
        }
        if (hatchGrabberState == HatchGrabberState.Driving)
        {
            double dist = ((startEncoderLeft - robotState.getLeftDriveEncoder())
                    + (startEncoderRight - robotState.getRightDriveEncoder())) / 2;
            if ((setPoint.armAngle() > 0 && dist > DRIVE_DIST) || (setPoint.armAngle() < 0 && dist < -DRIVE_DIST))
                hatchGrabberState = HatchGrabberState.Complete;
            else
                output = getGrabSetPoint(setPoint);
        }
        if (hatchGrabberState == HatchGrabberState.Complete)
        {
            output = setPoint;
        }

        lastOutput = output;
        return output;
    }

    private IManipulatorSetPoint getPushSetPoint(IManipulatorSetPoint setPoint)
    {
        if (setPoint == ManipulatorSetPoint.hatch_out_back || setPoint == ManipulatorSetPoint.hatch_out_front)
        {
            return getGrabOnlySetPoint(setPoint);
        }
        return new ManualManipulatorSetPoint(setPoint.armAngle() - (ARM_PUSH * Math.signum(setPoint.armAngle())),
                setPoint.wristAngle(), setPoint.elevatorHeight(), setPoint.frontFlipper(), setPoint.backFlipper());
    }

    private IManipulatorSetPoint getGrabSetPoint(IManipulatorSetPoint setPoint)
    {
        return new ManualManipulatorSetPoint(setPoint.armAngle() - (ARM_PUSH * Math.signum(setPoint.armAngle())),
                setPoint.wristAngle(), setPoint.elevatorHeight() + ELEV_LIFT, setPoint.frontFlipper(),
                setPoint.backFlipper());
    }

    private IManipulatorSetPoint getGrabOnlySetPoint(IManipulatorSetPoint setPoint)
    {
        return new ManualManipulatorSetPoint(setPoint.armAngle(), setPoint.wristAngle(),
                setPoint.elevatorHeight() + ELEV_LIFT, setPoint.frontFlipper(), setPoint.backFlipper());
    }

    public void Reset()
    {
        hatchGrabberState = HatchGrabberState.Off;
        startEncoderLeft = 0;
        startEncoderRight = 0;
        releaseTimer = 0;
    }

    public boolean IsActive()
    {
        return !(hatchGrabberState == HatchGrabberState.Off || hatchGrabberState == HatchGrabberState.Complete);
    }

    @Override
    public IManipulatorSetPoint GetLastOutput()
    {
        return lastOutput;
    }

    public HatchGrabberState getState()
    {
        return hatchGrabberState;
    }

}