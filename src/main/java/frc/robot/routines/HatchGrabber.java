package frc.robot.routines;

import frc.robot.RobotState;
import frc.robot.constants.IManipulatorSetPoint;
import frc.robot.constants.ManipulatorSetPoint;
import frc.robot.constants.ManualManipulatorSetPoint;

public class HatchGrabber extends ManipulatorRoutineBase
{
    public static final double ELEV_LIFT = 3;
    public static final double DRIVE_DIST = 2;

    public enum HatchGrabberState
    {
        Off, Grabbing, Driving, Complete
    }

    private HatchGrabberState hatchGrabberState = HatchGrabberState.Off;
    private double startEncoderLeft = 0;
    private double startEncoderRight = 0;
    private IManipulatorSetPoint lastOutput;

    public IManipulatorSetPoint GetPickupSetPoint(ManipulatorSetPoint setPoint)
    {
        IManipulatorSetPoint output = null;
        if (setPoint == null)
        {
            lastOutput = null;
            return null;
        }

        if (hatchGrabberState == HatchGrabberState.Off && !robotState.isSpearsClosed())
        {
            hatchGrabberState = HatchGrabberState.Grabbing;
        }
        if (hatchGrabberState == HatchGrabberState.Grabbing)
        {
            output = getGrabSetPoint(setPoint);
            if (onTarget())
            {
                hatchGrabberState = HatchGrabberState.Driving;
                startEncoderLeft = robotState.getLeftDriveEncoder();
                startEncoderRight = robotState.getRightDriveEncoder();
            }
        }
        if (hatchGrabberState == HatchGrabberState.Driving)
        {
            double dist = ((startEncoderLeft - robotState.getLeftDriveEncoder()) + (startEncoderRight - robotState.getRightDriveEncoder())) / 2;
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

    private IManipulatorSetPoint getGrabSetPoint(IManipulatorSetPoint setPoint)
    {
        return new ManualManipulatorSetPoint(setPoint.armAngle(), setPoint.wristAngle(),
                setPoint.elevatorHeight() + ELEV_LIFT, setPoint.frontFlipper(), setPoint.backFlipper());
    }

    public void Reset()
    {
        hatchGrabberState = HatchGrabberState.Off;
        startEncoderLeft = 0;
        startEncoderRight = 0;
    }

    public boolean IsActive()
    {
        return !(hatchGrabberState == HatchGrabberState.Off || hatchGrabberState == HatchGrabberState.Complete);
    }

    @Override
    public IManipulatorSetPoint GetLastOutput()
    {
        return null;
    }

}