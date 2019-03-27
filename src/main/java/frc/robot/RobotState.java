package frc.robot;

import frc.robot.constants.ManipulatorSetPoint;

public class RobotState
{
    private static RobotState mRobotState;

    public static RobotState getInstance()
    {
        if (mRobotState == null)
            mRobotState = new RobotState();
        return mRobotState;
    }

    private double leftDriveEncoder = 0;
    private double rightDriveEncoder = 0;

    private double armPosition = 0;
    private double wristPosition = 0;
    private double elevatorPosition = 0;

    private double frontFlipperPosition = 0;
    private double backFlipperPosition = 0;

    private boolean leftLimitSwitch = false;
    private boolean rightLimitSwitch = false;
    private ManipulatorSetPoint commandedSetPoint = null;

    public boolean isLeftLimitSwitch() {
        return this.leftLimitSwitch;
    }

    public boolean getLeftLimitSwitch() {
        return this.leftLimitSwitch;
    }

    public void setLeftLimitSwitch(boolean leftLimitSwitch) {
        this.leftLimitSwitch = leftLimitSwitch;
    }

    public boolean isRightLimitSwitch() {
        return this.rightLimitSwitch;
    }

    public boolean getRightLimitSwitch() {
        return this.rightLimitSwitch;
    }

    public void setRightLimitSwitch(boolean rightLimitSwitch) {
        this.rightLimitSwitch = rightLimitSwitch;
    }

    public ManipulatorSetPoint getCommandedSetPoint() {
        return this.commandedSetPoint;
    }

    public void setCommandedSetPoint(ManipulatorSetPoint commandedSetPoint) {
        this.commandedSetPoint = commandedSetPoint;
    }

    public double getFrontFlipperPosition() {
        return this.frontFlipperPosition;
    }

    public void setFrontFlipperPosition(double frontFlipperPosition) {
        this.frontFlipperPosition = frontFlipperPosition;
    }

    public double getBackFlipperPosition() {
        return this.backFlipperPosition;
    }

    public void setBackFlipperPosition(double backFlipperPosition) {
        this.backFlipperPosition = backFlipperPosition;
    }

    private double heading = 0;

    private boolean spearsClosed = false;

    public double getLeftDriveEncoder()
    {
        return this.leftDriveEncoder;
    }

    public void setLeftDriveEncoder(double leftDriveEncoder)
    {
        this.leftDriveEncoder = leftDriveEncoder;
    }

    public double getRightDriveEncoder()
    {
        return this.rightDriveEncoder;
    }

    public void setRightDriveEncoder(double rightDriveEncoder)
    {
        this.rightDriveEncoder = rightDriveEncoder;
    }

    public double getArmPosition()
    {
        return this.armPosition;
    }

    public void setArmPosition(double armPosition)
    {
        this.armPosition = armPosition;
    }

    public double getWristPosition()
    {
        return this.wristPosition;
    }

    public void setWristPosition(double wristPosition)
    {
        this.wristPosition = wristPosition;
    }

    public double getElevatorPosition()
    {
        return this.elevatorPosition;
    }

    public void setElevatorPosition(double elevatorPosition)
    {
        this.elevatorPosition = elevatorPosition;
    }

    public double getHeading()
    {
        return this.heading;
    }

    public void setHeading(double heading)
    {
        this.heading = heading;
    }

    public boolean isSpearsClosed()
    {
        return this.spearsClosed;
    }

    public void setSpearsClosed(boolean spearsClosed)
    {
        this.spearsClosed = spearsClosed;
    }
}