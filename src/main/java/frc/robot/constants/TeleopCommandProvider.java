package frc.robot.constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.hotteam67.HotController;
import org.hotteam67.HotLogger;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Flipper;

public class TeleopCommandProvider
{
    private final HotController driver;
    private final HotController operator;

    private double armReZeroTimer = 0;
    private double armReZeroCount = 50;

    private ManipulatorSetPoint outputSetPoint = null;
    private double LeftDrive = 0;
    private double RightDrive = 0;
    private double LeftDriveSteeringAssist = 0;
    private double RightDriveSteeringAssist = 0;
    private double HDrive = 0;
    private boolean intakeSolenoid = false;
    private boolean score = false;
    private boolean intakeOut = false;
    private boolean intakeIn = false;
    private boolean climb = false;
    private boolean hatchPickup = false;
    private boolean steeringAssist = false;

    boolean upPrev = false;
    boolean downPrev = false;

    private boolean limitSwitchPickup = false;
    private boolean limitSwitchPlace = false;

    private int frontFlipperCount = 0;
    private int backFlipperCount = 0;

    boolean commandToBack = true;
    boolean flipButtonPrevious = false;

    public TeleopCommandProvider(HotController driver, HotController operator)
    {
        this.driver = driver;
        this.operator = operator;
    }

    public void Reset()
    {
        armReZeroTimer = 0;
        armReZeroCount = 50;

        outputSetPoint = null;
        LeftDrive = 0;
        RightDrive = 0;
        LeftDriveSteeringAssist = 0;
        RightDriveSteeringAssist = 0;
        HDrive = 0;
        intakeSolenoid = false;
        score = false;
        intakeOut = false;
        intakeIn = false;
        climb = false;
        hatchPickup = false;
        steeringAssist = false;

        upPrev = false;
        downPrev = false;

        limitSwitchPickup = false;
        limitSwitchPlace = false;

        frontFlipperCount = 0;
        backFlipperCount = 0;

        commandToBack = true;
        flipButtonPrevious = false;
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

    public double LeftDriveSteeringAssist()
    {
        return LeftDriveSteeringAssist;
    }

    public double RightDriveSteeringAssist()
    {
        return RightDriveSteeringAssist;
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

    public void Update()
    {
        ManipulatorSetPoint frontTargetPosition = null;
        ManipulatorSetPoint backTargetPosition = null;

        boolean isLeftTriggerPressed = (operator.getLeftTrigger() >= .25);
        boolean isRightTriggerPressed = (operator.getRightTrigger() >= .25);

        if (operator.getButtonX())
        {
            frontTargetPosition = ManipulatorSetPoint.carry_front;
            backTargetPosition = ManipulatorSetPoint.carry_back;
        }
        if (operator.getButtonA())
        {
            if (!operator.getButtonBack())
            {
                frontTargetPosition = ManipulatorSetPoint.hatch_pickup_front;
                backTargetPosition = ManipulatorSetPoint.hatch_low_back;
            }
            else
            {
                frontTargetPosition = ManipulatorSetPoint.cargo_rocketLow_front;
                backTargetPosition = ManipulatorSetPoint.cargo_rocketLow_back;
            }
        }
        if (operator.getButtonB())
        {
            if (!operator.getButtonBack())
            {
                frontTargetPosition = ManipulatorSetPoint.hatch_mid_front;
                backTargetPosition = ManipulatorSetPoint.hatch_mid_back;
            }
            else
            {
                frontTargetPosition = ManipulatorSetPoint.cargo_rocketMid_front;
                backTargetPosition = ManipulatorSetPoint.cargo_rocketMid_back;
            }
        }
        if (operator.getButtonY())
        {
            if (!operator.getButtonBack())
            {
                frontTargetPosition = ManipulatorSetPoint.hatch_high_front;
                backTargetPosition = ManipulatorSetPoint.hatch_high_back;
            }
            else
            {
                frontTargetPosition = ManipulatorSetPoint.cargo_rocketHigh_front;
                backTargetPosition = ManipulatorSetPoint.cargo_rocketHigh_back;
            }
        }
        if (operator.getButtonLeftBumper())
        {
            frontTargetPosition = ManipulatorSetPoint.cargo_station_front;
            backTargetPosition = ManipulatorSetPoint.cargo_station_back;
        }
        if (operator.getButtonRightBumper())
        {
            frontTargetPosition = ManipulatorSetPoint.cargo_shuttle_front;
            backTargetPosition = ManipulatorSetPoint.cargo_shuttle_back;
        }
        if (isLeftTriggerPressed && !isRightTriggerPressed)
        {
            frontTargetPosition = backTargetPosition = ManipulatorSetPoint.climb_prep;
        }
        else if (isRightTriggerPressed && isLeftTriggerPressed)
        {
            frontTargetPosition = backTargetPosition = ManipulatorSetPoint.climber_down;
        }
        else if (isRightTriggerPressed && !isLeftTriggerPressed)
        {
            frontTargetPosition = backTargetPosition = ManipulatorSetPoint.climber_on;
        }

        climb = driver.getButtonB();

        if (operator.getButtonRightStick())
        {
            frontTargetPosition = ManipulatorSetPoint.cargo_pickup_front;
            backTargetPosition = ManipulatorSetPoint.cargo_pickup_back;
        }
        if (operator.getButtonStart())
        {
            armReZeroTimer++;
        }
        else
            armReZeroTimer = 0;

        if (driver.getButtonRightBumper())
        {
            intakeOut = false;
            intakeIn = true;

        }
        if (driver.getButtonLeftBumper())
        {
            intakeOut = true;
            intakeIn = false;
        }
        if (!driver.getButtonLeftBumper() && !driver.getButtonRightBumper())
        {
            intakeOut = false;
            intakeIn = false;
        }

        hatchPickup = driver.getButtonY();
        if (driver.getButtonX())
        {
            intakeSolenoid = true;
        }
        else if (driver.getButtonY())
        {
            intakeSolenoid = false;
        }

        if (driver.getButtonStart())
        {
            steeringAssist = true;
        }
        else
        {
            steeringAssist = false;
        }

        limitSwitchPlace = driver.getButtonRightStick();
        limitSwitchPickup = driver.getButtonLeftStick();
        score = driver.getButtonA();

        if (driver.getButtonBack() && !flipButtonPrevious)
        {
            commandToBack = !commandToBack;
        }
        flipButtonPrevious = driver.getButtonBack();

        outputSetPoint = (commandToBack) ? backTargetPosition : frontTargetPosition;

        turnDrive = ((driver.getStickRX() * .5));
        RightDrive = driver.getStickLY();
        LeftDrive = driver.getStickLY();
        RightDriveSteeringAssist = driver.getStickLY();
        LeftDriveSteeringAssist = driver.getStickLY();
        HDrive = ((driver.getRawAxis(3) - driver.getRawAxis(2)) / 2.0);

        boolean up = operator.getPOV() == 180;
        boolean down = operator.getPOV() == 0;

        if (up && !upPrev)
        {
            if (outputSetPoint != null && outputSetPoint.frontFlipper() == FlipperConstants.HATCH_FRONT)
                frontFlipperCount++;
            else if (outputSetPoint != null && outputSetPoint.backFlipper() == FlipperConstants.HATCH_BACK)
            {
                backFlipperCount++;
            }
        }
        else if (down && !downPrev)
        {
            if (outputSetPoint != null && outputSetPoint.frontFlipper() == FlipperConstants.HATCH_FRONT)
                frontFlipperCount--;
            else if (outputSetPoint != null && outputSetPoint.backFlipper() == FlipperConstants.HATCH_BACK)
            {
                backFlipperCount--;
            }
        }

        LogValues();

        upPrev = operator.getPOV() == 180;
        downPrev = operator.getPOV() == 0;

        if (rumble)
        {
            driver.setRumble(RumbleType.kLeftRumble, .25);
            driver.setRumble(RumbleType.kRightRumble, .25);
        }
        else
        {
            driver.setRumble(RumbleType.kLeftRumble, 0);
            driver.setRumble(RumbleType.kRightRumble, 0);
        }

        rumble = false;
    }

    public int FrontFlipperBumpCount()
    {
        return frontFlipperCount;
    }

    public int BackFlipperBumpCount()
    {
        return backFlipperCount;
    }

    private String setPointName(ManipulatorSetPoint setPoint)
    {
        if (setPoint != null)
            return setPoint.name();
        else
            return "null";
    }

    private void LogValues()
    {
        Log("outputSetPoint", setPointName((ManipulatorSetPoint) outputSetPoint));
        Log("frontFlipperCount", frontFlipperCount);
        Log("backFlipperCount", backFlipperCount);
    }

    public static final List<String> LoggerTags = new ArrayList<>(
            Arrays.asList("outputSetPoint", "frontFlipperCount", "backFlipperCount"));

    private void Log(String tag, double value)
    {
        Log(tag, String.valueOf(value));
    }

    private void Log(String tag, String value)
    {
        SmartDashboard.putString(tag, value);
        HotLogger.Log(tag, value);
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
        return armReZeroTimer >= armReZeroCount;
    }

    public boolean LimitSwitchScore()
    {
        return limitSwitchPlace;
    }

    public boolean LimitSwitchPickup()
    {
        return limitSwitchPickup;
    }

    private double turnDrive = 0;

    public double TurnDrive()
    {
        return turnDrive;
    }

    boolean rumble = false;

    public void Rumble()
    {
        rumble = true;
    }

    public void SetSpearsClosed(boolean b)
    {
        intakeSolenoid = b;
    }
}