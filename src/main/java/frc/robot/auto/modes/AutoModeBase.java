package frc.robot.auto.modes;

import frc.robot.RobotCommandProvider;

public abstract class AutoModeBase extends RobotCommandProvider
{
    public abstract boolean IsComplete();
}