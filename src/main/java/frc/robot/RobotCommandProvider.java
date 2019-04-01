package frc.robot;

import org.hotteam67.HotController;
import frc.robot.constants.ManipulatorSetPoint;

public abstract class RobotCommandProvider
{
    protected ManipulatorSetPoint outputSetPoint = null;
    protected double LeftDrive = 0;
    protected double RightDrive = 0;
    protected double HDrive = 0;
    protected double turnDrive = 0;
    protected boolean intakeSolenoid = false;
    protected boolean score = false;
    protected boolean intakeOut = false;
    protected boolean intakeIn = false;
    protected boolean climb = false;
    protected boolean hatchPickup = false;
    protected boolean steeringAssist = false;

    protected boolean limitSwitchPickup = false;
    protected boolean limitSwitchPlace = false;

    protected int frontFlipperCount = 0;
    protected int backFlipperCount = 0;
    protected boolean zeroWrist = false;
    protected boolean rumble = false;
    protected boolean armReZero;

    public abstract void Update();

    public boolean ZeroWrist()
    {
        return zeroWrist;
    }

    public void Reset()
    {
        zeroWrist = false;

        outputSetPoint = null;
        LeftDrive = 0;
        RightDrive = 0;
        HDrive = 0;
        score = false;
        intakeOut = false;
        intakeIn = false;
        climb = false;
        hatchPickup = false;
        steeringAssist = false;

        limitSwitchPickup = false;
        limitSwitchPlace = false;

        frontFlipperCount = 0;
        backFlipperCount = 0;
    }

    public ManipulatorSetPoint ManipulatorSetPoint()
    {
        return outputSetPoint;
    }

    public boolean ManipulatorScore()
    {
        return score;
    }

    public double LeftDrive()
    {
        return LeftDrive;
    }

    public double RightDrive()
    {
        return RightDrive;
    }

    public double HDrive()
    {
        return HDrive;
    }

    public boolean SpearsClosed()
    {
        return intakeSolenoid;
    }

    public boolean steeringAssistActivated()
    {
        return steeringAssist;
    }

    public int FrontFlipperBumpCount()
    {
        return frontFlipperCount;
    }

    public int BackFlipperBumpCount()
    {
        return backFlipperCount;
    }

    public boolean IntakeOut()
    {
        return intakeOut;
    }

    public boolean IntakeIn()
    {
        return intakeIn;
    }

    public boolean ClimberDeploy()
    {
        return climb;
    }

    public boolean HatchPickup()
    {
        return hatchPickup;
    }

    public boolean ARMREZERO()
    {
        return armReZero;
    }

    public boolean LimitSwitchScore()
    {
        return limitSwitchPlace;
    }

    public boolean LimitSwitchPickup()
    {
        return limitSwitchPickup;
    }

    public double TurnDrive()
    {
        return turnDrive;
    }
    
    public void Rumble()
    {
        rumble = true;
    }

    public void SetSpearsClosed(boolean b)
    {
        intakeSolenoid = b;
    }
}