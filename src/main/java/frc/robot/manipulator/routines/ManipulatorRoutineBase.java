package frc.robot.manipulator.routines;

import frc.robot.RobotState;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.IManipulatorSetPoint;
import frc.robot.constants.ManualManipulatorSetPoint;
import frc.robot.constants.WristConstants;

abstract class ManipulatorRoutineBase
{
    protected final RobotState robotState;
    public ManipulatorRoutineBase()
    {
        robotState = RobotState.getInstance();
    }

    public boolean onTarget(IManipulatorSetPoint setPoint)
    {
        return (Math.abs(setPoint.armAngle() - robotState.getArmPosition()) <= ArmConstants.allowableErrorDegrees
                && Math.abs(
                        setPoint.wristAngle() - robotState.getWristPosition()) <= WristConstants.allowableErrorDegrees
                && Math.abs(setPoint.elevatorHeight()
                        - robotState.getElevatorPosition()) <= ElevatorConstants.allowableErrorInches);
    }

    public boolean onTarget(double arm, double wrist, double elevator)
    {
        return onTarget(new ManualManipulatorSetPoint(arm, wrist, elevator, 0, 0));
    }

    public boolean onTarget()
    {
        if (GetLastOutput() != null)
            return onTarget(GetLastOutput());
        else return true;
    }

    public abstract boolean IsActive();
    public abstract IManipulatorSetPoint GetLastOutput();
    public abstract void Reset();
}