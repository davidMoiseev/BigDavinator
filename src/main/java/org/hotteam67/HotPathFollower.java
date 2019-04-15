/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.hotteam67;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.followers.DistanceFollower;

/**
 * Add your docs here.
 */
public class HotPathFollower
{
    public static final List<String> LoggerValues = new ArrayList<>(Arrays.asList("Path Points", "Path Heading",
            "Heading Error", "Turn Output", "Left Path Position", "Left Path Velocity", "Left Path Acceleration",
            "Left Path X", "Left Path Y", "Left Path Calculated Output", "Left Path Heading", "Right Path Position",
            "Right Path Velocity", "Right Path Acceleration", "Right Path X", "Right Path Y",
            "Right Path Calculated Output", "Right Path Heading"));

    // Encoder ticks per revolution on either side
    private final double ticksPerRev;
    // Wheel diameter on either side
    private final double wheelDiameter;

    // Current overall state of the HotPathFollower
    State pathState = State.Disabled;

    private int lastPathFollowed = -1;

    private static class Follower
    {
        public final DistanceFollower LeftFollower;
        public final DistanceFollower RightFollower;

        public Follower()
        {
            LeftFollower = new DistanceFollower();
            RightFollower = new DistanceFollower();
        }
    }

    private List<Follower> loadedPaths = new ArrayList<>();

    // PIDFA constants for the position error. Stored for holding last point only
    private double POS_P = 0, POS_I = 0, POS_D = 0, POS_V = 0, POS_A = 0;
    // PID constants for the angle error. Stored for holding last point only
    private double ANGLE_P = 0;

    // Whether we are running in reverse
    private boolean isInverted = false;

    private Segment segLeftPrev;

    private Segment segRightPrev;

    /**
     * State of the follower. Disabled - not yet run, or has been reset Enabled -
     * currently running/producing output Holding - holding the last point of the
     * path Complete - has reached the configured acceptable target
     */
    public enum State
    {
        Disabled, Enabled, Complete
    }

    /**
     * Path output class, just returns the right and left desired outputs for the
     * path
     */
    public static class Output
    {
        public final double Left;
        public final double Right;
        public final double Turn;

        public Output(double l, double r, double turn)
        {
            this.Left = l;
            this.Right = r;
            this.Turn = turn;
        }
    }

    /**
     * Constructor sets up HotPathFollower without a default loaded path
     * 
     * @param ticksPerRev
     * @param wheelDiameter
     */
    public HotPathFollower(double ticksPerRev, double wheelDiameter)
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
    public HotPathFollower(double ticksPerRev, double wheelDiameter, Path[] paths)
    {
        this.ticksPerRev = ticksPerRev;
        this.wheelDiameter = wheelDiameter;
        LoadPaths(paths);
    }

    public void LoadPaths(Path[] paths)
    {
        System.out.println("Start Path Load");
        loadedPaths = new ArrayList<>();
        if (paths == null || paths.length == 0)
            return;

        for (int i = 0; i < paths.length; ++i)
        {
            Follower f = new Follower();
            try
            {
                if (new File(paths[i].Left).exists())
                    f.LeftFollower.setTrajectory(Pathfinder.readFromCSV(new File(paths[i].Left)));
                else
                    return;
            }
            catch (Exception e)
            {
                e.printStackTrace();
                return;
            }

            try
            {
                if (new File(paths[i].Right).exists())
                    f.RightFollower.setTrajectory(Pathfinder.readFromCSV(new File(paths[i].Right)));
                else
                    return;
            }
            catch (Exception e)
            {
                e.printStackTrace();
                return;
            }
            ConfigFollower(f.RightFollower, f.LeftFollower);
            loadedPaths.add(f);
        }
    }

    /**
     * Whether to invert the encoder values, gyro, and output to run the robot
     * backwards
     * 
     * @param inverted
     */
    public void SetInverted(boolean inverted)
    {
        isInverted = inverted;
    }

    private int GetPolarity()
    {
        return (isInverted) ? -1 : 1;
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

        if (loadedPaths != null && loadedPaths.size() > 0)
        {
            loadedPaths.forEach(x -> ConfigFollower(x.LeftFollower, x.RightFollower));
        }
    }

    private void ConfigFollower(DistanceFollower leftFollower, DistanceFollower rightFollower)
    {
        if (leftFollower != null)
        {
            leftFollower.configurePIDVA(POS_P, POS_I, POS_D, POS_V, POS_A);
        }
        if (rightFollower != null)
        {
            rightFollower.configurePIDVA(POS_P, POS_I, POS_D, POS_V, POS_A);
        }
    }

    /**
     * Configure the angle P constant
     * 
     * @param p
     */
    public void ConfigAngleP(double p)
    {
        ANGLE_P = p;
    }

    public int getPoints()
    {
        return points;
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
    int points = 0;

    public Output FollowNextPoint(int pathNumber, double currentPositionLeft, double currentPositionRight,
            double currentHeading)
    {
        double turn = 0;
        if (pathNumber != lastPathFollowed)
        {
            points = 0;
            lastPathFollowed = pathNumber;
            pathState = State.Enabled;
        }
        points++;
        currentPositionLeft = ((Math.PI * wheelDiameter) / ticksPerRev) * currentPositionLeft * GetPolarity();
        currentPositionRight = ((Math.PI * wheelDiameter) / ticksPerRev) * currentPositionRight * GetPolarity();

        if (loadedPaths == null || pathNumber >= loadedPaths.size())
            return new Output(0, 0, 0);

        DistanceFollower leftFollower = loadedPaths.get(pathNumber).LeftFollower;
        DistanceFollower rightFollower = loadedPaths.get(pathNumber).RightFollower;

        if (leftFollower == null || rightFollower == null)
            return new Output(0, 0, 0);
        double l = 0, r = 0;
        Segment segLeft = null;
        Segment segRight = null;

        // Path is not currently being followed, so start it
        if (pathState == State.Disabled)
            pathState = State.Enabled;

        // Path is enabled and the points are not yet complete
        if (pathState == State.Enabled && (!leftFollower.isFinished() || !rightFollower.isFinished()))
        {
            l = leftFollower.calculate(currentPositionLeft) * GetPolarity();
            r = rightFollower.calculate(currentPositionRight) * GetPolarity();

            if (leftFollower.isFinished())
            {
                segLeft = segLeftPrev;
            }
            else
            {
                segLeft = leftFollower.getSegment();
            }

            if (rightFollower.isFinished())
            {
                segRight = segRightPrev;
            }
            else
            {
                segRight = rightFollower.getSegment();
            }

            Log("Left", segLeft, l);
            Log("Right", segRight, r);

            // Add angle error, in degrees
            double targetHeading = -Pathfinder.boundHalfDegrees(Pathfinder.r2d(segLeft.heading));
            double headingError = Pathfinder.boundHalfDegrees(targetHeading - currentHeading);
            turn = ANGLE_P * headingError;
            HotLogger.Log("Heading Error", headingError);
            HotLogger.Log("Turn Output", turn);

            segLeftPrev = segLeft;
            segRightPrev = segRight;
        }

        // We are done
        else
            pathState = State.Complete;
        // SmartDashboard.putNumber("complete", 1);

        // We are there, no output
        if (pathState == State.Complete)
        {
            l = 0;
            r = 0;
            turn = 0;
        }

        return new Output(l, r, turn);
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
        if (loadedPaths != null && loadedPaths.size() > 0)
        {
            for (int i = 0; i < loadedPaths.size(); ++i)
            {
                if (loadedPaths.get(i).LeftFollower != null)
                    loadedPaths.get(i).LeftFollower.reset();
                if (loadedPaths.get(i).RightFollower != null)
                    loadedPaths.get(i).RightFollower.reset();
            }
        }
        pathState = State.Disabled;
    }

    public void ClearPaths()
    {
        loadedPaths.clear();
    }

    /**
     * Private logging function, logs for the path given
     * 
     * @param motorName
     *                      Left or Right Motor
     * @param s
     *                      the segment to get values from for logging
     * @param output
     *                      the calculated output to the motor
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
        HotLogger.Log(motorName + " Path Heading", Pathfinder.boundHalfDegrees(Pathfinder.r2d(s.heading)));
    }
}
