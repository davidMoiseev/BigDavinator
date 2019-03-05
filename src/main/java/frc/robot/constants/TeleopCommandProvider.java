package frc.robot.constants;

import org.hotteam67.HotController;

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

    private double frontFlipperSetPointAdjust = 0;
    private double backFlipperSetPointAdjust = 0;

    private boolean lastFlipperUp = false;
    private boolean lastFlipperDown = false;

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

        if (operator.getDpad() > 0 && !lastFlipperUp)
        {
            if (outputSetPoint.frontFlipper() == FlipperConstants.HATCH_FRONT)
            {
                frontFlipperSetPointAdjust += 2;
            }
            else if (outputSetPoint.backFlipper() == FlipperConstants.HATCH_BACK)
            {
                backFlipperSetPointAdjust += 2;
            }
        }
        else if (operator.getDpad() < 0 && !lastFlipperDown)
        {
            if (outputSetPoint.frontFlipper() == FlipperConstants.HATCH_FRONT)
            {
                frontFlipperSetPointAdjust -= 2;
            }
            else if (outputSetPoint.backFlipper() == FlipperConstants.HATCH_BACK)
            {
                backFlipperSetPointAdjust -= 2;
            }
        }

        fixFlipper();

        lastFlipperDown = (operator.getDpad() < 0);
        lastFlipperUp = (operator.getDpad() > 0);

    }

    private void fixFlipper()
    {
        if (outputSetPoint.frontFlipper() == FlipperConstants.HATCH_FRONT)
        {
            outputSetPoint = new ManualManipulatorSetPoint(outputSetPoint.armAngle(), outputSetPoint.wristAngle(),
                outputSetPoint.elevatorHeight(), outputSetPoint.frontFlipper() - frontFlipperSetPointAdjust,
                outputSetPoint.backFlipper());
        }
        else if (outputSetPoint.backFlipper() == FlipperConstants.HATCH_BACK)
        {
            outputSetPoint = new ManualManipulatorSetPoint(outputSetPoint.armAngle(), outputSetPoint.wristAngle(),
                outputSetPoint.elevatorHeight(), outputSetPoint.frontFlipper(),
                outputSetPoint.backFlipper() - backFlipperSetPointAdjust);
        }
    }

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
}