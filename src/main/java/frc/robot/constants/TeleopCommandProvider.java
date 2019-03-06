package frc.robot.constants;

import org.hotteam67.HotController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Flipper;

public class TeleopCommandProvider implements IRobotCommandProvider
{
    private final HotController driver;
    private final HotController operator;

    private IManipulatorSetPoint outputSetPoint = null;
    private double LeftDrive = 0;
    private double RightDrive = 0;
    private double HDrive = 0;
    private boolean intakeSolenoid = false;
    private boolean score = false;
    private boolean intakeOut = false;
    private boolean intakeIn = false;
    private boolean climb = false;
    private boolean hatchPickup = false;

    private double frontFlipperSetPointAdjust = 0;
    private double backFlipperSetPointAdjust = 0;

    private boolean lastFlipperUp = false;
    private boolean lastFlipperDown = false;

    private final double flipper_add = 4;
    private double frontFlipperCount = 0;
    private double backFlipperCount = 0;

    public TeleopCommandProvider(HotController driver, HotController operator)
    {
        this.driver = driver;
        this.operator = operator;
    }

    @Override
    public IManipulatorSetPoint ManipulatorSetPoint()
    {
        return outputSetPoint;
    }

    @Override
    public boolean ManipulatorScore()
    {
        return score;
    }

    @Override
    public double LeftDrive()
    {
        return LeftDrive;
    }

    @Override
    public double RightDrive()
    {
        return RightDrive;
    }

    @Override
    public double HDrive()
    {
        return HDrive;
    }

    @Override
    public boolean IntakeSolenoid()
    {
        return intakeSolenoid;
    }

    boolean commandToBack = true;
    boolean flipButtonPrevious = false;
    private boolean allowClimbMotors = false;

    @Override
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
                frontTargetPosition = ManipulatorSetPoint.hatch_low_front;
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
            /*
             * frontTargetPosition = ManipulatorSetPoint.cargo_rocketLow_front;
             * backTargetPosition = ManipulatorSetPoint.cargo_rocketLow_back;
             */
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

        if (operator.getButtonLeftStick())
        {
            /*
             * frontTargetPosition = ManipulatorSetPoint.cargo_pickup_front;
             * backTargetPosition = ManipulatorSetPoint.cargo_pickup_back;
             */
        }
        if (operator.getButtonRightStick())
        {
            frontTargetPosition = ManipulatorSetPoint.cargo_pickup_front;
            backTargetPosition = ManipulatorSetPoint.cargo_pickup_back;
        }

        if (driver.getButtonRightBumper())
        {
            intakeOut = true;
            intakeIn = false;

        }
        if (driver.getButtonLeftBumper())
        {
            intakeOut = false;
            intakeIn = true;
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

        score = driver.getButtonA();

        if (operator.getButtonLeftStick() && !flipButtonPrevious)
        {
            commandToBack = !commandToBack;
        }
        flipButtonPrevious = operator.getButtonLeftStick();

        outputSetPoint = (commandToBack) ? backTargetPosition : frontTargetPosition;

        RightDrive = -driver.getStickLY() - (driver.getStickRX() * .5);
        LeftDrive = -driver.getStickLY() + (driver.getStickRX() * .5);
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

        SmartDashboard.putBoolean("frontCarry",
                outputSetPoint != null && outputSetPoint.frontFlipper() == FlipperConstants.HATCH_FRONT);
        SmartDashboard.putBoolean("backCarry",
                outputSetPoint != null && outputSetPoint.backFlipper() == FlipperConstants.HATCH_BACK);

        SmartDashboard.putNumber("frontFlipperCount", frontFlipperCount);
        SmartDashboard.putNumber("backFlipperCount", backFlipperCount);

        SmartDashboard.putNumber("operatorPOV", operator.getPOV());
        upPrev = operator.getPOV() == 180;
        downPrev = operator.getPOV() == 0;
        SmartDashboard.putBoolean("upPrev", upPrev);
        SmartDashboard.putBoolean("downPrev", downPrev);

        if (outputSetPoint != null)
        {
            if (outputSetPoint.frontFlipper() == FlipperConstants.HATCH_FRONT)
                outputSetPoint = new ManualManipulatorSetPoint(outputSetPoint.armAngle(), outputSetPoint.wristAngle(),
                        outputSetPoint.elevatorHeight(), outputSetPoint.frontFlipper() + (frontFlipperCount * 3),
                        outputSetPoint.backFlipper());
            else if (outputSetPoint.backFlipper() == FlipperConstants.HATCH_BACK)
                outputSetPoint = new ManualManipulatorSetPoint(outputSetPoint.armAngle(), outputSetPoint.wristAngle(),
                        outputSetPoint.elevatorHeight(), outputSetPoint.frontFlipper(),
                        outputSetPoint.backFlipper() + (backFlipperCount * 3));
        }
    }

    boolean upPrev = false;
    boolean downPrev = false;

    @Override
    public boolean IntakeOut()
    {
        return intakeOut;
    }

    @Override
    public boolean IntakeIn()
    {
        return intakeIn;
    }

    @Override
    public boolean ClimberDeploy()
    {
        return climb;
    }

    @Override
    public boolean HatchPickup()
    {
        return hatchPickup;
    }
}