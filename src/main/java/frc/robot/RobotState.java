package frc.robot;

import frc.robot.constants.IManipulatorSetPoint;
import frc.robot.constants.ManipulatorSetPoint;
import frc.robot.manipulator.routines.HatchGrabber.HatchGrabberState;
import frc.robot.manipulator.routines.HatchPlacer.HatchPlacerState;

public class RobotState
{
    private static RobotState mRobotState;
    
    public static RobotState getInstance()
    {
        if (mRobotState == null)
            mRobotState = new RobotState();
        return mRobotState;
    }

    /**
     * @return the drivePitch
     */
    public double getDrivePitch()
    {
        return drivePitch;
    }

    /**
     * @param drivePitch
     *                       the drivePitch to set
     */
    public void setPitch(double drivePitch)
    {
        this.drivePitch = drivePitch;
    }

    /**
     * @return the climberDeployed
     */
    public boolean isClimberDeployed()
    {
        return climberDeployed;
    }

    /**
     * @param climberDeployed
     *                            the climberDeployed to set
     */
    public void setClimberDeployed(boolean climberDeployed)
    {
        this.climberDeployed = climberDeployed;
    }

    /**
     * @return the turnSpeed
     */
    public double getTurnSpeed()
    {
        return turnSpeed;
    }

    /**
     * @param turnSpeed
     *                      the turnSpeed to set
     */
    public void setTurnSpeed(double turnSpeed)
    {
        this.turnSpeed = turnSpeed;
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

    private boolean climberDeployed = false;

    private double drivePitch = 0;

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
    private double turnSpeed = 0;

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
         * @return the armIsBack
         */
        public boolean isArmIsBack()
        {
            return armIsBack;
        }

        /**
         * @param armIsBack
         *                      the armIsBack to set
         */
        public void setArmIsBack(boolean armIsBack)
        {
            this.armIsBack = armIsBack;
        }

        /**
         * @return the visionCanSeeTarget
         */
        public boolean isVisionCanSeeTarget()
        {
            return visionCanSeeTarget;
        }

        /**
         * @param visionCanSeeTarget
         *                               the visionCanSeeTarget to set
         */
        public void setVisionCanSeeTarget(boolean visionCanSeeTarget)
        {
            this.visionCanSeeTarget = visionCanSeeTarget;
        }

        /**
         * @return the visionTurnAtTarget
         */
        public boolean isVisionTurnAtTarget()
        {
            return visionTurnAtTarget;
        }

        /**
         * @param visionTurnAtTarget
         *                               the visionTurnAtTarget to set
         */
        public void setVisionTurnAtTarget(boolean visionTurnAtTarget)
        {
            this.visionTurnAtTarget = visionTurnAtTarget;
        }

        /**
         * @return the visionAtTarget
         */
        public boolean isVisionDistanceAtTarget()
        {
            return visionDistanceAtTarget;
        }

        /**
         * @param visionAtTarget
         *                           the visionAtTarget to set
         */
        public void setVisionDistanceAtTarget(boolean visionAtTarget)
        {
            this.visionDistanceAtTarget = visionAtTarget;
        }

        /**
         * @return the hatchPlacerState
         */
        public HatchPlacerState getHatchPlacerState()
        {
            return hatchPlacerState;
        }

        /**
         * @param hatchPlacerState
         *                             the hatchPlacerState to set
         */
        public void setHatchPlacerState(HatchPlacerState hatchPlacerState)
        {
            this.hatchPlacerState = hatchPlacerState;
        }

        /**
         * @return the hatchGrabberState
         */
        public HatchGrabberState getHatchGrabberState()
        {
            return hatchGrabberState;
        }

        /**
         * @param hatchGrabberState
         *                              the hatchGrabberState to set
         */
        public void setHatchGrabberState(HatchGrabberState hatchGrabberState)
        {
            this.hatchGrabberState = hatchGrabberState;
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

        private HatchGrabberState hatchGrabberState = HatchGrabberState.Off;
        private HatchPlacerState hatchPlacerState = HatchPlacerState.Off;

        private boolean visionDistanceAtTarget = false;
        private boolean visionTurnAtTarget = false;
        private boolean visionCanSeeTarget = false;
        private boolean armIsBack = true;
    }
}