package frc.robot;

import org.hotteam67.HotController;
import frc.robot.constants.ManipulatorSetPoint;

public abstract class RobotCommandProvider
{

    protected double manualWrist = 0;
    public double ManualWrist()
    {
        return manualWrist;
    }

    protected ManipulatorSetPoint outputSetPoint = null;
    protected double LeftDrive = 0;
    protected double RightDrive = 0;
    protected double HDrive = 0;
    protected double turnDrive = 0;
    protected boolean intakeSolenoid = false;
    protected boolean manipulatorScore = false;
    protected boolean intakeOut = false;
    protected boolean intakeIn = false;
    protected boolean climb = false;
    protected boolean hatchPickup = false;
    protected boolean steeringAssist = false;
    protected boolean visionDrive = false;
    protected boolean useAutoPipeline = false;

    public boolean UseAutoPipeLine()
    {
        return useAutoPipeline;
    }

    public boolean VisionDrive()
    {
        return visionDrive;
    }

    protected boolean limitSwitchPickup = false;
    protected boolean limitSwitchPlace = false;

    protected int frontFlipperCount = 0;
    protected int backFlipperCount = 0;
    protected boolean zeroWrist = false;
    protected boolean rumble = false;
    protected boolean armReZero;
    protected boolean useManualWrist = false;

    public abstract void Update();
    public Integer ActivePath()
    {
        return -1;
    }

    public boolean ZeroWrist()
    {
        return zeroWrist;
    }

    public boolean UseManualWrist()
    {
        return useManualWrist;
    }

    public void Reset()
    {
        zeroWrist = false;
        manualWrist = 0;
        useManualWrist = false;

        outputSetPoint = null;
        LeftDrive = 0;
        RightDrive = 0;
        HDrive = 0;
        manipulatorScore = false;
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
        return manipulatorScore;
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