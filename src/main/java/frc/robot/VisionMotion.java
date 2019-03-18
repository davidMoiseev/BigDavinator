/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FTC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.hotteam67.HotPathFollower;

//import org.graalvm.compiler.nodeinfo.StructuralInput.Memory;

//import static org.junit.Assert.assertArrayEquals;

//import com.sun.org.apache.xpath.internal.axes.SelfIteratorNoPredicate;
//import com.sun.tools.javac.jvm.Target;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionMotion
{
    Vision vision = new Vision();

    private final double p = .015;
    public static final double MIN_REF_COUNT = 25;
    public static final double MAX_TURN = .4;
    public static final double MIN_TURN = .2;
    private int refCount = 0;
    private double yawTarget = 0;
    VisionState visionState = VisionState.GetReference;

    enum VisionState
    {
        GetReference, TurnToReference
    }

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
        double error = yawTarget - currentYaw;
        if (Math.abs(error) < 2 && refCount > MIN_REF_COUNT)
        {
            yawTarget = vision.getHeading();
            visionState = VisionState.TurnToReference;
            refCount = 0;
            error = yawTarget - currentYaw;
        }

        if (Math.abs(error) > 2)
        {
            double turn = p * error;
            if (Math.abs(turn) > MAX_TURN)
            {
                turn = Math.signum(turn) * MAX_TURN;
            }
            if (Math.abs(turn) < MIN_TURN)
            {
                turn = Math.signum(turn) * MIN_TURN;
            }

            return new Output(turn, -turn, 0);
        }
        return new Output(0, 0, 0);
    }
};