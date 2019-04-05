package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.hotteam67.HotController;
import org.hotteam67.HotLogger;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotCommandProvider;
import frc.robot.constants.FlipperConstants;
import frc.robot.constants.ManipulatorSetPoint;

public class TeleopCommandProvider extends RobotCommandProvider
{
    private double armReZeroTimer = 0;
    private double armReZeroCount = 50;
    boolean commandToBack = true;
    boolean flipButtonPrevious = false;
    boolean upPrev = false;
    boolean downPrev = false;
    int zeroWristTimer = 0;

    private final HotController driver;
    private final HotController operator;
    private boolean rumbleOperator = false;

    public TeleopCommandProvider(HotController driver, HotController operator)
    {
        this.driver = driver;
        this.operator = operator;
    }

    @Override
    public void Reset()
    {
        armReZeroTimer = 0;
        armReZeroCount = 50;
        zeroWristTimer = 0;
        upPrev = false;
        downPrev = false;
        commandToBack = true;
        flipButtonPrevious = false;
        super.Reset();
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

        armReZero = armReZeroTimer > armReZeroCount;

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
        /*
         * if (driver.getButtonY()) { intakeSolenoid = false; }
         */

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
        manipulatorScore = driver.getButtonA();

        if (driver.getButtonBack() && !flipButtonPrevious)
        {
            commandToBack = !commandToBack;
        }
        flipButtonPrevious = driver.getButtonBack();

        outputSetPoint = (commandToBack) ? backTargetPosition : frontTargetPosition;

        turnDrive = ((driver.getStickRX() * .5));
        RightDrive = driver.getStickLY();
        LeftDrive = driver.getStickLY();
        HDrive = ((driver.getRawAxis(3) - driver.getRawAxis(2)) / 2.0);

        boolean up = operator.getPOV() == 180;
        boolean down = operator.getPOV() == 0;
        boolean right = operator.getPOV() == 90;
        boolean left = operator.getPOV() == 45;

        SmartDashboard.putBoolean("RIGHT", right);
        SmartDashboard.putBoolean("LEFT", left);

        if (right)
        {
            useManualWrist = true;
            manualWrist = -operator.getStickRX();
        }
        else
        {
            manualWrist = 0;
            useManualWrist = false;
        }

        if (left)
        {
            useManualWrist = true;
            if (zeroWristTimer > 50)
            {
                rumbleOperator = true;
                zeroWrist = true;
            }
            else
            {
                zeroWristTimer++;
            }
        }
        else
        {
            zeroWristTimer = 0;
            zeroWrist = false;
        }

        ManipulatorSetPoint currentSetPoint = RobotState.Actions.getInstance().getDesiredManipulatorSetPoint();
        if (up && !upPrev)
        {
            if (currentSetPoint != null && currentSetPoint.frontFlipper() == FlipperConstants.HATCH_FRONT)
                frontFlipperCount++;
            else if (currentSetPoint != null && currentSetPoint.backFlipper() == FlipperConstants.HATCH_BACK)
            {
                backFlipperCount++;
            }
        }
        else if (down && !downPrev)
        {
            if (currentSetPoint != null && currentSetPoint.frontFlipper() == FlipperConstants.HATCH_FRONT)
                frontFlipperCount--;
            else if (currentSetPoint != null && currentSetPoint.backFlipper() == FlipperConstants.HATCH_BACK)
            {
                backFlipperCount--;
            }
        }

        LogValues();

        upPrev = operator.getPOV() == 180;
        downPrev = operator.getPOV() == 0;

        if (rumble)
        {
            driver.setRumble(RumbleType.kLeftRumble, .4);
            driver.setRumble(RumbleType.kRightRumble, .4);
        }
        else
        {
            driver.setRumble(RumbleType.kLeftRumble, 0);
            driver.setRumble(RumbleType.kRightRumble, 0);
        }
        if (rumbleOperator)
        {
            operator.setRumble(RumbleType.kLeftRumble, .4);
            operator.setRumble(RumbleType.kRightRumble, .4);
        }
        else
        {
            operator.setRumble(RumbleType.kLeftRumble, 0);
            operator.setRumble(RumbleType.kRightRumble, 0);
        }

        rumble = false;
        rumbleOperator = false;
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

    private String setPointName(ManipulatorSetPoint setPoint)
    {
        if (setPoint != null)
            return setPoint.name();
        else
            return "null";
    }
}