/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FTC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import org.graalvm.compiler.nodeinfo.StructuralInput.Memory;

//import static org.junit.Assert.assertArrayEquals;

//import com.sun.org.apache.xpath.internal.axes.SelfIteratorNoPredicate;
//import com.sun.tools.javac.jvm.Target;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.WiringIDs;

public class VisionMotion
{
    Vision backCamera = new Vision("limelight-back", Vision.X_BACK, Vision.HEIGHT_BACK);
    Vision frontCamera = new Vision("limelight-front", Vision.X_FRONT, Vision.HEIGHT_FRONT);

    public double h;
    public double speedH;
    public static final double pGainH = 0.04;
    public static final double iGainH = 0.000008;
    public double speed = 0;
    public double Lspeed;
    public double Rspeed;
    public double pAngle;
    public double iAngle;
    public double p;
    public double i;
    public double motorOutput;
    public int num;
    public double Houtput;
    public double Loutput;
    public double Routput;
    public double gyroLoutput;
    public double gyroRoutput;
    public double gyroHoutput;
    public double bouncePrevY = 0;
    public double earlierCanSeeTarget = 0.0;
    public double prevHeadingVis = 0.0;
    public double targetAngle;
    public double targetVisDistance = 17.0; // inches, 30.125 + distance btwn robot perimeter and camera, aka constant C
    public double distanceHorizontal;
    public double angle2;
    public double angle1;
    public double distance = 0;
    public double currentYaw;
    public double referenceAngle;
    private int target;

    /*
     * Auto-Align Members
     */
    public static final double TURN_P = .0065;
    public static final double MIN_REF_COUNT = 0;
    public static final double MAX_TURN = .2;
    public static final double MIN_TURN = .055;
    public static final double MAX_ACCEL = .015;

    private int turn_referenceAngleCount = 0;
    private Double turn_referenceAngle = null;
    private double turn_previousOutput = 0;
    private boolean turn_hasReset = true;

    public static class Output
    {
        public final double Left;
        public final double Right;
        public final double HDrive;

        public Output(double left, double right, double hDrive)
        {
            Left = left;
            Right = right;
            HDrive = hDrive;
        }
    }

    public Output autoAlign(double currentYaw)
    {
        getCamera().setPipeline(2);
        double error;
        if (!WiringIDs.IS_PRACTICE_BOT)
            currentYaw = -currentYaw;
        turn_referenceAngleCount++;
        if ((turn_hasReset || turn_referenceAngleCount > MIN_REF_COUNT) && canSeeTarget())
        {
            turn_referenceAngle = getCamera().getHeading() + currentYaw;
            turn_referenceAngleCount = 0;
            error = turn_referenceAngle - currentYaw;
        }

        if (turn_referenceAngle == null)
            return new Output(0, 0, 0);

        error = turn_referenceAngle - currentYaw;

        // Temporary testing things
        //SmartDashboard.putNumber("currentYaw", currentYaw);
        //SmartDashboard.putNumber("error", error);

        if (Math.abs(error) > 2)
        {
            double turn = TURN_P * error;

            // Do max/min turn with signs
            if (Math.abs(turn) > MAX_TURN)
            {
                turn = Math.signum(turn) * MAX_TURN;
            }
            if (Math.abs(turn) < MIN_TURN)
            {
                turn = Math.signum(turn) * MIN_TURN;
            }

            double absTurn = Math.abs(turn);
            double absPrev = Math.abs(turn_previousOutput);
            if (absTurn > absPrev && absTurn - absPrev > MAX_ACCEL)
            {
                absTurn = absPrev + MAX_ACCEL;
                turn = absTurn * Math.signum(turn);
            }
            turn_previousOutput = turn;
            return new Output(-turn, turn, 0);
        }
        turn_previousOutput = 0;
        return new Output(0, 0, 0);
    }

    public double getDist()
    {
        getCamera().setPipeline(1);
        return getCamera().findDistance();
    }

    public void clearPipeline()
    {
        frontCamera.setPipeline(0);
        backCamera.setPipeline(0);
    }

    public boolean canSeeTarget()
    {
        return getCamera().canSeeTarget() == 1;
    }

    public void resetVision()
    {
        turn_hasReset = true;
        turn_referenceAngle = null;
        clearPipeline();
    }

    public double findProportionalSkew(double targetSkew)
    {
        double error = getCamera().getSkew() - targetSkew;
        return error;
    }

    public boolean definitelySeesTarget()
    {
        double maxHeadingChange = 1000;
        if ((earlierCanSeeTarget == getCamera().getTV())
                && (Math.abs(prevHeadingVis - getCamera().getHorizontal()) < maxHeadingChange))
        {
            earlierCanSeeTarget = getCamera().getTV();
            prevHeadingVis = getCamera().getHeading();
            return true;
        }
        else
        {
            earlierCanSeeTarget = getCamera().getTV();
            prevHeadingVis = getCamera().getHeading();
            return false;
        }

    }

    public double shuffleVisionPID()
    {
        pAngle = findProportionalSkew(0);
        double Hspeed = (pAngle * pGainH) + (iAngle * iGainH);
        if (Hspeed > 0.5)
            Hspeed = 0.5;
        else if (Hspeed < -0.5)
        {
            Hspeed = -0.5;
        }
        // driveTrain.basicSideOutput(Hspeed);
        if (this.definitelySeesTarget() == true)
        {
            return Hspeed;
        }
        else
        {
            return 0;
        }
    }

    public void setPipeline(double pipeline)
    {
        getCamera().setPipeline(pipeline);
    }

    public void sendAngle(double yaw)
    {
        currentYaw = yaw;

    }

    public boolean getHigh()
    { // true = high, false = low
        if (getCamera().getVertical() > 6)
        {
            return true;
        }
        else if (getCamera().getVertical() < 3)
        {
            return false;
        }
        else if (getCamera().getVertical() > 3 && getCamera().getVertical() < 6)
        {
            if (getCamera().getTA() < 5)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    public int selectTarget(double pipeline)
    {
        if (pipeline == 0)
        {
            target = 67;
        }
        else
        {
            if ((currentYaw < (Math.PI / 8)) && (currentYaw > (Math.PI / -8)))
            { // if angle is zero
                referenceAngle = 0;
                target = 0; // front cargo ship

            }
            else if ((currentYaw > (3 * Math.PI / 8)) && (currentYaw < (5 * Math.PI / -8)))
            { // if angle is pi/2
                referenceAngle = Math.PI / 2;
                if (getHigh() == true)
                { // if it is high
                    target = 1; // right rocket center
                }
                else if (getHigh() == false)
                { // if it is low
                    target = 2; // left side cargo
                }
            }
            else if ((currentYaw < (-3 * Math.PI / 8)) && (currentYaw > (-5 * Math.PI / -8)))
            { // if angle is -pi/2
                referenceAngle = Math.PI / -2;
                if (getHigh() == true)
                { // if it is high
                    target = 3; // left rocket center
                }
                else if (getHigh() == false)
                { // if it is low
                    target = 4; // right side cargo
                }
            }
            else if ((currentYaw > (Math.PI / 8)) && (currentYaw < (3 * Math.PI / 8)))
            { // if angle is pi/4
                referenceAngle = Math.PI / 4;
                target = 5; // right rocket near

            }
            else if ((currentYaw < (-Math.PI / 8)) && (currentYaw > (-3 * Math.PI / 8)))
            { // if angle is -pi/4
                referenceAngle = Math.PI / -4;
                target = 6; // left rocket near

            }
            else if ((currentYaw > (5 * Math.PI / 8)) && (currentYaw < (7 * Math.PI / 8)))
            { // if angle is 3pi/4
                referenceAngle = 3 * Math.PI / 4;
                target = 7; // right rocket far
            }

            else if ((currentYaw < (-5 * Math.PI / 8)) && (currentYaw > (-7 * Math.PI / 8)))
            { // if angle is -3pi/4
                referenceAngle = -3 * Math.PI / 4;
                target = 8; // left rocket far
            }

            else if ((currentYaw > (Math.PI / -8)) && (currentYaw < (Math.PI / 8)))
            { // if angle is zero
                referenceAngle = (Math.PI);
                target = 0; // front cargo ship
            }
        }
        //SmartDashboard.putNumber("referenceAngle", referenceAngle);
        return target;
    }

    public double driveToVisionDistance(double pipeline)
    {
        if (getCamera().findDistance(this.selectTarget(pipeline)) < 20.0)
        {
            motorOutput = 0.0;
            return motorOutput;
        }
        else
        {
            motorOutput = 0.4;
            return motorOutput;
        }
    }

    public double getReferenceAngle()
    {
        return referenceAngle;
    }

    public void targetLineUp()
    { // faster but sloppier than gyroTargetLineUp
        this.shuffleVisionPID();
        // this.turnVision();
        // this.driveToVisionDistance();
        // double Lspeed = this.driveToVisionDistance() + (this.turnVision());
        // double Rspeed = this.driveToVisionDistance() - (this.turnVision());
        if (Lspeed > 1.0)
        {
            Lspeed = 1.0;
        }
        else if (Lspeed < -1.0)
        {
            Lspeed = -1.0;
        }
        if (Rspeed > 1.0)
        {
            Rspeed = 1.0;
        }
        else if (Rspeed < -1.0)
        {
            Rspeed = -1.0;
        }
        if (this.definitelySeesTarget() == true)
        {
            Loutput = Lspeed;
            Routput = Rspeed;
        }
    }

    public double outputL()
    { // for targetLineUp
        this.targetLineUp();
        double outputL = Loutput;
        return outputL;
    }

    public double outputR()
    { // for targetLineUp
        this.targetLineUp();
        double outputR = Routput;
        return outputR;
    }

    public void getTargetAngle()
    {
        targetAngle = referenceAngle + Math.toRadians(getCamera().getHeading());
    }

    public boolean setGyroLineUpVars(double pipeline)
    {
        distance = getCamera().findDistance(target);
        distanceHorizontal = distance * Math.tan(targetAngle);
        angle2 = Math.atan((distance - targetVisDistance) / distanceHorizontal);
        angle1 = ((Math.PI / 2) - targetAngle) - angle2;
        SmartDashboard.putNumber("distance", distance);
        return true;
    }

    public boolean whichCamera()
    {
        return true;
    }

    public void gyroTargetLineUp(double yaw, double vt)
    { // vt is the motor output of the resultant
      // vy = Math.tan(yaw * getCamera().getHeading()) * ;//Math.sin(yaw) * vt;
      // vx = //Math.cos(yaw)* vt;
      // vxh = vx * Math.cos(yaw);
      // vxlr = vx * Math.sin(yaw);
      // vyh = vy * Math.sin(yaw);
      // vylr = vy * Math.cos(yaw);
      // gyroLoutput = vy;//(vylr + vxlr);
      // gyroRoutput = vy;//(vylr + vxlr);
      // gyroHoutput = vx;//(vyh + vxh);
        if (whichCamera() == true)
        { // front camera, no limelight
            gyroLoutput = -(distance * vt) / (distance + Math.abs(distanceHorizontal));
            gyroRoutput = (distance * vt) / (distance + Math.abs(distanceHorizontal));
            // gyroHoutput = -.1 * vx;// (distanceHorizontal * vt) / (distanceHorizontal +
            // distance);
        }
        else
        { // back camera, limelight
            gyroLoutput = (distance * vt) / (distance + Math.abs(distanceHorizontal));
            gyroRoutput = -(distance * vt) / (distance + Math.abs(distanceHorizontal));
            // gyroHoutput = -.1 * vx;// (distanceHorizontal * vt) / (distanceHorizontal +
            // distance);
        }

        SmartDashboard.putNumber("vt", vt);
    }

    public boolean targetReached(double distanceTarget, double pipeline)
    {
        // distanceDiagonal = Math.sqrt((distance * distance) + (distanceHorizontal *
        // distanceHorizontal));
        if (getCamera().findDistance(this.selectTarget(pipeline)) <= distanceTarget)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public double outputGyroL(double yaw, double maxMotorOutput)
    {
        double outputL = gyroLoutput;
        return outputL;
    }

    public double outputGyroR(double yaw, double maxMotorOutput)
    {
        double outputR = gyroRoutput;
        return outputR;
    }

    public double outputGyroH(double yaw, double maxMotorOutput)
    {
        double outputH = gyroHoutput;
        return outputH;
    }

    public boolean detectBallBounce()
    {
        if (Math.abs(getCamera().getVertical() - bouncePrevY) > 5.0)
        {
            bouncePrevY = getCamera().getVertical();
            return true;
        }
        else
        {
            bouncePrevY = getCamera().getVertical();
            return false;
        }
    }

    public void writeDashBoardVis()
    {
        /*
        SmartDashboard.putNumber("AAA DIST FRONT", frontCamera.findDistance());
        SmartDashboard.putNumber("AAA DIST BACK", backCamera.findDistance());
        SmartDashboard.putNumber("AAA ANGLE FRONT", frontCamera.angleFromCenter());
        SmartDashboard.putNumber("AAA ANGLE BACK", backCamera.angleFromCenter());
        */
    }

    boolean useBackCamera = false;
	public void useBackCamera(boolean b)
	{
        useBackCamera = b;
    }
    
    private Vision getCamera()
    {
        return (useBackCamera) ? backCamera : frontCamera;
    }

	public void UpdateRobotState()
	{
        getCamera().getNetworkTables();
        RobotState robotState = RobotState.getInstance();

        robotState.setVisionArea(getCamera().getTA());
        robotState.setVisionX(getCamera().getHorizontal());
        robotState.setVisionY(getCamera().getVertical());
	}
};