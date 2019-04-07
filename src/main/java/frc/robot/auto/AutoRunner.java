package frc.robot.auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotCommandProvider;
import frc.robot.auto.modes.AutoModeBase;
import frc.robot.auto.modes.RocketAuto;
import frc.robot.Paths;

public class AutoRunner
{
    public static enum Auto
    {
        RocketHatchLeft(() -> new RocketAuto(35, Paths.RHRB1)),
        RocketHatchRight(() -> new RocketAuto(-35, Paths.LHRB1));

        public final Supplier<AutoModeBase> Initializer;
        private Auto(Supplier<AutoModeBase> initializer)
        {
            this.Initializer = initializer;
        }
    }

    AutoModeBase auton = null;

    public static SendableChooser<Auto> BuildAutonChooser()
    {
        SendableChooser<Auto> autonChooser = new SendableChooser<>();
        for (Auto auto : Auto.values())
        {
            autonChooser.addOption(auto.name(), auto);
        }
        return autonChooser;
    }

    public void Select(Auto auto)
    {
        auton = auto.Initializer.get();
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
