/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.hotteam67;

import static org.junit.Assert.assertNotNull;

import java.io.File;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.followers.EncoderFollower;

/**
 * Add your docs here.
 */
public class HotPathFollower
{
    // Allowable angle error defaults to .5 degrees
    private double allowableAngleError = .5;
    // Allowable position error defaults to 2000 encoder ticks;
    private double allowablePosError = 2000;

    // Encoder ticks per revolution on either side
    private final int ticksPerRev;
    // Wheel diameter on either side
    private final double wheelDiameter;

    // Current overall state of the HotPathFollower
    State pathState = State.Disabled;

    // Jaci leftFollower will do most calculation and handle the left path
    private EncoderFollower leftFollower;
    // Jaci rightFollower will do most calculation and handle the right path
    private EncoderFollower rightFollower;

    // Whether to hold the last point of the path
    private boolean holdLastPoint = false;

    // PIDFA constants for the position error. Stored for holding last point only
    private double POS_P = 0, POS_I = 0, POS_D = 0, POS_V = 0, POS_A = 0;
    // PID constants for the angle error. Stored for holding last point only
    private double ANGLE_P = 0;

    // Previous segment, used for holding the output value of followers
    private Segment lastSegmentRight = null;
    // Previous segment, used for holding the output value of followers
    private Segment lastSegmentLeft = null;

    /**
     * State of the follower. Disabled - not yet run, or has been reset Enabled -
     * currently running/producing output Holding - holding the last point of the
     * path Complete - has reached the configured acceptable target
     */
    public enum State
    {
        Disabled, Enabled, Holding, Complete
    }

    /**
     * Path output class, just returns the right and left desired outputs for the
     * path
     */
    public static class Output
    {
        public final double Left;
        public final double Right;

        public Output(double l, double r)
        {
            this.Left = l;
            this.Right = r;
        }
    }

    /**
     * Constructor sets up HotPathFollower without a default loaded path
     * 
     * @param ticksPerRev
     * @param wheelDiameter
     */
    public HotPathFollower(int ticksPerRev, double wheelDiameter)
    {
        this.ticksPerRev = ticksPerRev;
        this.wheelDiameter = wheelDiameter;
    }

    /**
     * Constructor sets up HotPathFollower, and loads a path from disk right away
     * 
     * @param ticksPerRev
     * @param wheelDiameter
     */
    public HotPathFollower(int ticksPerRev, double wheelDiameter, String leftPathFile, String rightPathFile)
    {
        this.ticksPerRev = ticksPerRev;
        this.wheelDiameter = wheelDiameter;
        LoadPath(leftPathFile, rightPathFile);
    }

    /**
     * Load a path from disk, given csv files for left/right side, and create new
     * followers for it
     * 
     * @param leftPathFile
     *                          the fully qualified path to the left csv file
     * @param rightPathFile
     *                          the fully qualified path to the right csv file
     */
    public void LoadPath(String leftPathFile, String rightPathFile)
    {
        // Try catch for IOExceptions
        try
        {
            leftFollower = new EncoderFollower(Pathfinder.readFromCSV(new File(leftPathFile)));
            leftFollower.configureEncoder(0, ticksPerRev, wheelDiameter);
        }
        catch (Exception e)
        {
            leftFollower = null;
            e.printStackTrace();
            System.out.println("Failed to load left path");
        }

        // Try catch for IOExceptions
        try
        {
            rightFollower = new EncoderFollower(Pathfinder.readFromCSV(new File(rightPathFile)));
            rightFollower.configureEncoder(0, ticksPerRev, wheelDiameter);
        }
        catch (Exception e)
        {
            rightFollower = null;
            e.printStackTrace();
            System.out.println("Failed to load right path");
        }
        // Give PIDVA to followers again
        ConfigPosPIDVA(POS_P, POS_I, POS_D, POS_V, POS_A);
        // New path so reset everything
        Reset();
    }

    /**
     * Configure all of the constants for position, with acceleration constant
     * defaulting to 0
     * 
     * @param p
     * @param i
     * @param d
     * @param v
     */
    public void ConfigPosPIDV(double p, double i, double d, double v)
    {
        ConfigPosPIDVA(p, i, d, v, 0);
    }

    /**
     * Configure all of the constants for position
     * 
     * @param p
     * @param i
     * @param d
     * @param v
     * @param a
     */
    public void ConfigPosPIDVA(double p, double i, double d, double v, double a)
    {
        POS_P = p;
        POS_I = i;
        POS_D = d;
        POS_V = v;
        POS_A = a;
        if (leftFollower != null)
        {
            leftFollower.configurePIDVA(p, i, d, v, a);
        }
        if (rightFollower != null)
        {
            rightFollower.configurePIDVA(p, i, d, v, a);
        }
    }

    /**
     * Configure the angle P constant
     * 
     * @param p
     */
    public void ConfigAnglePID(double p)
    {
        ANGLE_P = p;
    }

    /**
     * Config allowable error for holding mode Pos
     * 
     * @param error
     */
    public void ConfigAllowablePosError(double error)
    {
        allowableAngleError = error;
    }

    /**
     * Config allowable error for holding mos Angle
     * 
     * @param error
     */
    public void ConfigAllowableAngleError(double error)
    {
        allowableAngleError = error;
    }

    /**
     * Config whether to hold the last point of the profile. Will just complete at
     * the last point if true
     * 
     * @param doHold
     */
    public void SetHoldLastPointEnabled(boolean doHold)
    {
        this.holdLastPoint = doHold;
    }

    /**
     * Get the output of the path follower with given inputs for the next point.
     * Should be called at the same frequency as loaded path
     * 
     * @param currentPositionLeft
     *                                 the current position, in encoder ticks, of
     *                                 left motor
     * @param currentPositionRight
     *                                 the current position, in encoder ticks, of
     *                                 right motor
     * @param currentHeading
     *                                 the current heading of the robot
     * @return an output object with desired left/right outputs, scaled as a double
     *         from -1 to 1 for -100% output to 100% output
     */
    public Output FollowNextPoint(int currentPositionLeft, int currentPositionRight, double currentHeading)
    {
        if (leftFollower == null || rightFollower == null)
            return new Output(0, 0);
        double l = 0, r = 0;

        // Path is not currently being followed, so start it
        if (pathState == State.Disabled)
            pathState = State.Enabled;

        // Path is enabled and the points are not yet complete
        if (pathState == State.Enabled && !leftFollower.isFinished() || !rightFollower.isFinished())
        {
            l = leftFollower.calculate(currentPositionLeft);
            r = rightFollower.calculate(currentPositionRight);
            try
            {
                lastSegmentLeft = leftFollower.getSegment();
                lastSegmentRight = rightFollower.getSegment();
            }
            catch (Exception ignored)
            {
            }

            // Add angle error, in degrees
            double targetHeading = Pathfinder.boundHalfDegrees(360 - Pathfinder.r2d(lastSegmentLeft.heading));
            double headingError = Pathfinder.boundHalfDegrees(targetHeading - currentHeading);
            double turn = ANGLE_P * headingError;

            l += turn;
            r -= turn;
        }

        // Points are complete so make sure the state is holding, if its enabled
        else if (holdLastPoint)
            pathState = State.Holding;

        // If holding is not enabled, we are done
        else
            pathState = State.Complete;

        // Hold the last point, or check if we are within configured acceptable range
        if (pathState == State.Holding)
        {
            double ticksPerMeter = (ticksPerRev / (Math.PI * wheelDiameter));

            // Error in meters
            double leftError = lastSegmentLeft.position - (currentPositionLeft / ticksPerMeter);
            double rightError = lastSegmentRight.position - (currentPositionRight / ticksPerMeter);

            // Error in degrees
            double targetHeading = Pathfinder.boundHalfDegrees(360 - Pathfinder.r2d(lastSegmentLeft.heading));
            double headingError = Pathfinder.boundHalfDegrees(targetHeading - currentHeading);

            // Check for allowable error deadband
            if (Math.abs(leftError) < allowablePosError && Math.abs(rightError) < allowablePosError
                    && Math.abs(headingError) < allowableAngleError)
            {
                pathState = State.Complete;
            }
            // Manually calculate with just P
            else
            {
                l = leftError * POS_P;
                r = rightError * POS_P;
                double turn = ANGLE_P * headingError;
                l += turn;
                r -= turn;
            }
        }

        // We are there, no output
        if (pathState == State.Complete)
        {
            l = 0;
            r = 0;
        }

        Log("Left", lastSegmentLeft, l);
        Log("Right", lastSegmentRight, r);

        return new Output(l, r);
    }

    /**
     * Get the current state of the follower
     * 
     * @return the current state of the follower
     */
    public State GetState()
    {
        return pathState;
    }

    /**
     * Reset the state and calculations of the follower
     */
    public void Reset()
    {
        if (leftFollower != null && rightFollower != null)
        {
            leftFollower.reset();
            rightFollower.reset();
        }
        pathState = State.Disabled;
        lastSegmentLeft = null;
        lastSegmentRight = null;
    }

    /**
     * Private logging function, logs for the path given
     * @param motorName Left or Right Motor
     * @param s the segment to get values from for logging
     * @param output the calculated output to the motor
     */
    private static void Log(String motorName, Segment s, double output)
    {
        if (s == null)
            return;
        HotLogger.Log(motorName + " Path Position", s.position);
        HotLogger.Log(motorName + " Path Velocity", s.velocity);
        HotLogger.Log(motorName + " Path Acceleration", s.acceleration);
        HotLogger.Log(motorName + " Path X", s.x);
        HotLogger.Log(motorName + " Path Y", s.y);
        HotLogger.Log(motorName + " Path Calculated Output", output);
    }
}
