/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FTC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionMotion
{
    Vision vision = new Vision();

    private final double p = .007;
    public static final double MIN_REF_COUNT = 0;
    public static final double MAX_TURN = .2;
    public static final double MIN_TURN = .055;
    private int refCount = 0;
    private double yawTarget = 0;
    public static final double MAX_ACCEL = .015;
    double prevOutput = 0;
    private boolean hasResetVision = true;

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
        vision.setPipeline(1);
        double error = yawTarget - currentYaw;
        refCount++;
        if ((hasResetVision || refCount > MIN_REF_COUNT) && canSeeTarget())
        {
            yawTarget = vision.getHeading() + currentYaw;
            refCount = 0;
            error = yawTarget - currentYaw;
        }

        // Temporary testing things
        SmartDashboard.putNumber("currentYaw", currentYaw);
        SmartDashboard.putNumber("visionAngle", error);

        if (Math.abs(error) > 2)
        {
            double turn = p * error;

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
            double absPrev = Math.abs(prevOutput);
            if (absTurn > absPrev && absTurn - absPrev > MAX_ACCEL)
            {
                absTurn = absPrev + MAX_ACCEL;
                turn = absTurn * Math.signum(turn);
            }
            prevOutput = turn;
            return new Output(-turn, turn, 0);
        }
        prevOutput = 0;
        return new Output(0, 0, 0);
    }

    public double getDist()
    {
        vision.setPipeline(1);
        return vision.findDistance();
    }

    public void clearPipeline()
    {
        vision.setPipeline(2);
    }

    public boolean canSeeTarget()
    {
        return vision.canSeeTarget() == 1;
    }

    public void resetVision()
    {
        hasResetVision = true;
        clearPipeline();
    }
};