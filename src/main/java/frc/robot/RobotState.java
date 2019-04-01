package frc.robot;

import frc.robot.constants.IManipulatorSetPoint;
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

    private double visionX = 0;
    private double visionY = 0;
    private double visionArea = 0;

    /**
     * @return the visionArea
     */
    public double getVisionArea()
    {
        return visionArea;
    }

    /**
     * @param visionArea
     *                       the visionArea to set
     */
    public void setVisionArea(double visionArea)
    {
        this.visionArea = visionArea;
    }

    /**
     * @return the visionY
     */
    public double getVisionY()
    {
        return visionY;
    }

    /**
     * @param visionY
     *                    the visionY to set
     */
    public void setVisionY(double visionY)
    {
        this.visionY = visionY;
    }

    /**
     * @return the visionX
     */
    public double getVisionX()
    {
        return visionX;
    }

    /**
     * @param visionX
     *                    the visionX to set
     */
    public void setVisionX(double visionX)
    {
        this.visionX = visionX;
    }

    public boolean isLeftLimitSwitch()
    {
        return this.leftLimitSwitch;
    }

    public boolean getLeftLimitSwitch()
    {
        return this.leftLimitSwitch;
    }

    public void setLeftLimitSwitch(boolean leftLimitSwitch)
    {
        this.leftLimitSwitch = leftLimitSwitch;
    }

    public boolean isRightLimitSwitch()
    {
        return this.rightLimitSwitch;
    }

    public boolean getRightLimitSwitch()
    {
        return this.rightLimitSwitch;
    }

    public void setRightLimitSwitch(boolean rightLimitSwitch)
    {
        this.rightLimitSwitch = rightLimitSwitch;
    }

    public double getFrontFlipperPosition()
    {
        return this.frontFlipperPosition;
    }

    public void setFrontFlipperPosition(double frontFlipperPosition)
    {
        this.frontFlipperPosition = frontFlipperPosition;
    }

    public double getBackFlipperPosition()
    {
        return this.backFlipperPosition;
    }

    public void setBackFlipperPosition(double backFlipperPosition)
    {
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

    public static class Actions
    {
        private static Actions mInstance = null;
        public static Actions getInstance()
        {
            if (mInstance == null)
            {
                mInstance = new Actions();
            }
            return mInstance;
        }

        /**
         * @return the manipulatorMoving
         */
        public boolean isManipulatorMoving()
        {
            return manipulatorMoving;
        }

        /**
         * @param manipulatorMoving
         *                              the manipulatorMoving to set
         */
        public void setManipulatorMoving(boolean manipulatorMoving)
        {
            this.manipulatorMoving = manipulatorMoving;
        }

        /**
         * @return the commandedManipulatorSetPoint
         */
        public IManipulatorSetPoint getCommandedManipulatorSetPoint()
        {
            return commandedManipulatorSetPoint;
        }

        /**
         * @param commandedManipulatorSetPoint
         *                                         the commandedManipulatorSetPoint to
         *                                         set
         */
        public void setCommandedManipulatorSetPoint(IManipulatorSetPoint commandedManipulatorSetPoint)
        {
            this.commandedManipulatorSetPoint = commandedManipulatorSetPoint;
        }

        /**
         * @return the desiredManipulatorSetPoint
         */
        public ManipulatorSetPoint getDesiredManipulatorSetPoint()
        {
            return desiredManipulatorSetPoint;
        }

        /**
         * @param desiredManipulatorSetPoint
         *                                       the desiredManipulatorSetPoint to set
         */
        public void setDesiredManipulatorSetPoint(ManipulatorSetPoint desiredManipulatorSetPoint)
        {
            this.desiredManipulatorSetPoint = desiredManipulatorSetPoint;
        }

        private ManipulatorSetPoint desiredManipulatorSetPoint = null;
        private IManipulatorSetPoint commandedManipulatorSetPoint = null;

        private boolean manipulatorMoving = false;
    }
}