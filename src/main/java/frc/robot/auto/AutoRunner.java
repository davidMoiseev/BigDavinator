package frc.robot.auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotCommandProvider;
import frc.robot.auto.modes.AutoModeBase;
import frc.robot.auto.modes.RocketAuto;
import frc.robot.constants.WiringIDs;
import frc.robot.auto.modes.DoubleSideAuto;
import frc.robot.auto.modes.FrontSideAuto;
import frc.robot.auto.Paths;

public class AutoRunner
{
    public static enum Auto
    {
        RocketLeft(() -> new RocketAuto(20, -35, -90, -10)),
        RocketRight(() -> new RocketAuto(-20, 35, 90, 10)),
        RightFrontCargoOuter(() -> new FrontSideAuto(false, 32, 45, 15, 90)),
        LeftFrontCargoOuter(() -> new FrontSideAuto(false, -32, -45, -15, -90)),
        RightFrontCargoHeadOn(() -> new FrontSideAuto(true, 32, 45, 15, 90)),
        LeftFrontCargoHeadOn(() -> new FrontSideAuto(true, -32, -45, -15, -90)),
        LeftSideCargo(() -> new DoubleSideAuto(-15, 90, 20, 45)),
        RightSideCargo(() -> new DoubleSideAuto(15, -90, -55, -5));

        public final Supplier<AutoModeBase> Initializer;
        private Auto(Supplier<AutoModeBase> initializer)
        {
            this.Initializer = initializer;
        }
    }

    AutoModeBase auton = null;

    public void Select(Auto auto)
    {
        if (auto != null)
            auton = auto.Initializer.get();
        else
            auton = null;
    }

    public AutoModeBase Run()
    {
        if (auton != null)
            auton.Update();
        return auton;
    }

    public boolean AutoSelected()
    {
        return auton != null;
    }

    public boolean IsComplete()
    {
        if (auton != null)
            return auton.IsComplete();
        else
            return false;
    }
}
