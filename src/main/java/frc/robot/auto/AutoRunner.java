package frc.robot.auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotCommandProvider;
import frc.robot.auto.modes.AutoModeBase;
import frc.robot.auto.modes.DriveStraightAuto;
import frc.robot.auto.modes.FrontHatchAuto;

public class AutoRunner
{
    public static enum Auto
    {
        DriveStraight(() -> new DriveStraightAuto()),
        FrontHatch(() -> new FrontHatchAuto());

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
