package frc.robot.auto;

import frc.robot.RobotCommandProvider;
import frc.robot.auto.modes.AutoModeBase;
import frc.robot.auto.modes.DriveStraightAuto;

public class AutoRunner
{
    public static enum Auto
    {
        DriveStraight
    }

    AutoModeBase auton = null;

    public void Select(Auto auto)
    {
        switch (auto)
        {
        case DriveStraight:
            auton = new DriveStraightAuto();
            break;
        }
    }

    public RobotCommandProvider Run()
    {
        if (auton != null)
            auton.Update();
        return auton;
    }

    public boolean IsComplete()
    {
        return auton.IsComplete();
    }
}
