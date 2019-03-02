/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.hotteam67.HotController;
import org.hotteam67.HotLogger;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.WiringIDs;
import frc.robot.constants.IManipulatorSetPoint;
import frc.robot.constants.ManipulatorSetPoint;
import frc.robot.constants.ManualManipulatorSetPoint;

/**
 * Add your docs here.
 */
public class Manipulator
{

    private enum InitializationState
    {
        UNINITIALIZED, CALIBRATING, READY
    }

    private enum ManipulatorState
    {
        initializing, packagePosition, outOfPackagePosition, atTarget;
    }

    private enum RobotSide
    {
        FRONT, BACK
    }

    public static final List<String> LoggerTags = new ArrayList<>(Arrays.asList("AA debug Arm"));

    private Elevator elevator;
    private Intake intake;
    private IntakePneumatics pneumaticIntake;
    private Wrist wrist;
    private Arm arm;
    private Flipper frontFlipper;
    private Flipper backFlipper;
    private Solenoid climber;

    private HotController driver;
    private HotController operator;
    private ArmPigeon armPigeon;
    private DriveTrain drivetrain;
    private InitializationState initializationState;
    private ManipulatorState manipulatorState;
    private boolean startButtonPrevious = true;
    private boolean backButtonPrevious = true;
    private boolean commandToBack = false;

    private double prevElevHeight;
    private double prevArmAngle;
    private double prevWristAngle;

    private final double ELEVATOR_CLEAR_HEIGHT = 20;
    private final double OVER_THE_TOP_HEIGHT = 7;
    private final double ELEVATOR_TOLERANCE = 2;
    private final double ARM_TOLERANCE = 2; // need
    private final double WRIST_TOLERANCE = 2; // need
    private final double ARM_LENGTH = 21;
    private final double WRIST_LENGTH = 14;
    private final double ELEVATOR_LENGTH = 19;
    private final double FRAME_COLLISION = 5;
    private final double SUPPORTS_COLLISION = 14; // need?
    private final double FRAME_LENGTH = 16; // 14
    private final double intakeHeight = 13.5;
    private final double SCORE_DISTANCE = 5; // inches to move arm/intake forward for scoring

    public Manipulator(HotController operator, HotController driver, TalonSRX rightElevator, TalonSRX intake,
            DriveTrain drivetrain)
    {
        this.elevator = new Elevator(new TalonSRX(WiringIDs.LEFT_ELEVATOR), rightElevator);
        this.wrist = new Wrist(WiringIDs.WRIST);
        this.arm = new Arm(WiringIDs.SHOULDER);
        this.armPigeon = new ArmPigeon(WiringIDs.PIGEON_ARM);
        this.intake = new Intake(driver);
        this.pneumaticIntake = new IntakePneumatics(driver);
        this.frontFlipper = new Flipper(WiringIDs.FLIPPER_FRONT, false, -20, false);
        this.backFlipper = new Flipper(WiringIDs.FLIPPER_BACK, false, -20, true);

        this.operator = operator;
        this.driver = driver;
        this.drivetrain = drivetrain;

        RestartInitialization();
        manipulatorState = ManipulatorState.initializing;

    }

    public void InitializeTalons()
    {
        elevator.initialize();
        wrist.initialize();
        arm.initialize();
        frontFlipper.initialize();
        backFlipper.initialize();
    }

    public void RestartInitialization()
    {
        initializationState = InitializationState.UNINITIALIZED;
    }

    public boolean isReady()
    {
        return initializationState == InitializationState.READY;
    }

    public void RunManipulatorInitialization()
    {

        if (initializationState == InitializationState.UNINITIALIZED)
        {
            armPigeon.RestartCalibration();
            initializationState = InitializationState.CALIBRATING;
        }
        if (initializationState == InitializationState.CALIBRATING)
        {

            armPigeon.CalibratePigeon();
            if (armPigeon.PigeonReady())
            {
                initializationState = InitializationState.READY;
                manipulatorState = ManipulatorState.packagePosition;
                arm.setPosition(armPigeon.GetAngle());
                wrist.setPosition(armPigeon.GetAngle() - 134);
                elevator.zeroSensors();

            }
        }

    }

    public void DisplaySensors()
    {
        elevator.displaySensorsValue();
        arm.displaySensorsValue();
        wrist.displaySensorsValue();
        frontFlipper.displaySensorsValue();
        backFlipper.displaySensorsValue();
    }

    private void setTargetSafeElevator(IManipulatorSetPoint setPoint)
    {
        setTargetSafeElevator(setPoint.elevatorHeight(), setPoint.armAngle(), setPoint.wristAngle());
    }

    private void setTargetSafeElevator(double elevHeight, double armAngle, double wristAngle)
    {
        // Moving down, so wait for the arm first
        if (elevHeight < elevator.getPosition())
        {
            System.out.println("SAFE ELEVATOR");
            setTargets(prevElevHeight, armAngle, wristAngle);
            if (arm.reachedTarget() && wrist.reachedTarget())
            {
                setTargets(elevHeight, armAngle, wristAngle);
            }
        }
        // Might be moving the arm to a bad spot, so move the elevator first
        else
        {
            System.out.println("UNSAFE ELEVATOR");
            setTargets(elevHeight, armAngle, wristAngle);
        }
    }

    private void Control(IManipulatorSetPoint targetPosition)
    {

        if (manipulatorState == ManipulatorState.initializing)
        {
            elevator.disable();
            arm.disable();
            wrist.disable();
            frontFlipper.disable();
            backFlipper.disable();
        }
        else if (manipulatorState == ManipulatorState.packagePosition)
        {
            // elevator.disable();
            arm.disable();
            wrist.disable();
            frontFlipper.disable();
            backFlipper.disable();
            if (targetPosition != null)
            {
                manipulatorState = ManipulatorState.outOfPackagePosition;
            }
        }
        else if (manipulatorState == ManipulatorState.outOfPackagePosition)
        {
            double tmpArm = arm.getPosition();

            elevator.setTarget(ManipulatorSetPoint.firstPosition);
            /*
             * frontFlipper.setTarget(ManipulatorSetPoint.firstPosition);
             * backFlipper.setTarget(ManipulatorSetPoint.firstPosition);
             */
            if (elevator.reachedTarget())
            {
                arm.setTarget(tmpArm);
                wrist.setTarget(ManipulatorSetPoint.firstPosition);
                if (wrist.reachedTarget())
                {
                    arm.setTarget(ManipulatorSetPoint.firstPosition);
                    if (arm.reachedTarget())
                    {
                        manipulatorState = ManipulatorState.atTarget;
                    }
                }
            }
        }

        if (manipulatorState == ManipulatorState.atTarget)
        {
            if (willCollideWithFrame(targetPosition.elevatorHeight(), targetPosition.armAngle(),
                    targetPosition.wristAngle())
                    || willCollideWithSupports(targetPosition.elevatorHeight(), targetPosition.armAngle(),
                            targetPosition.wristAngle()))
            {

                elevator.disable();
                arm.disable();
                wrist.disable();
                HotLogger.Log("AA debug Arm", 0);
                System.out.println("LogStuff: 0");
            }
            // If it isn't
            else
            {
                // Same side
                if (getArmSide(targetPosition.armAngle()) == getArmSide(arm.getPosition()))
                {
                    // Front side
                    if (getArmSide(targetPosition.armAngle()) == RobotSide.FRONT)
                    {
                        // If arm is in the range where it won't collide with the frame or elevator
                        // supports
                        if (targetPosition.armAngle() > ManipulatorSetPoint.limit_front_high.armAngle()
                                && targetPosition.armAngle() < ManipulatorSetPoint.limit_front_low.armAngle())
                        {
                            setTargetSafeElevator(targetPosition);
                            HotLogger.Log("AA debug Arm", 3);
                            System.out.println("LogStuff: 3");
                        }
                        // compare current locations, if the target is out of bounds, set to a high
                        // value.
                        // if the current is about to collide, then for the supports, adjust the arm, if
                        // frame, the elev height
                        else
                        {
                            if (willCollideWithFrame(elevator.getPosition(), arm.getPosition(), wrist.getPosition())
                                    || (willCollideWithSupports(elevator.getPosition(), arm.getPosition(),
                                            wrist.getPosition())))
                            {
                                // setTargets(targetPosition, targetPosition, targetPosition);
                                setTargetSafeElevator(prevElevHeight, targetPosition.armAngle(),
                                        targetPosition.wristAngle());
                                HotLogger.Log("AA debug Arm", 4);
                                System.out.println("LogStuff: 4");
                            }
                            else
                            {
                                setTargetSafeElevator(targetPosition);
                                HotLogger.Log("AA debug Arm", 5);
                                System.out.println("LogStuff: 5");
                            }
                        }
                    }
                    // Back side
                    else
                    {
                        // If arm is in the range where it won't collide with the frame or elevator
                        // supports
                        if (targetPosition.armAngle() > ManipulatorSetPoint.limit_back_high.armAngle()
                                && targetPosition.armAngle() < ManipulatorSetPoint.limit_back_low.armAngle())
                        {
                            setTargetSafeElevator(targetPosition);
                            HotLogger.Log("AA debug Arm", 6);
                            System.out.println("LogStuff: 6");
                        }
                        // compare current locations, if the target is out of bounds, set to a high
                        // value.
                        // if the current is about to collide, then for the supports, adjust the arm, if
                        // frame, the elev height
                        else
                        {
                            if (willCollideWithFrame(elevator.getPosition(), arm.getPosition(), wrist.getPosition())
                                    || (willCollideWithSupports(elevator.getPosition(), arm.getPosition(),
                                            wrist.getPosition())))
                            {
                                setTargetSafeElevator(prevElevHeight, targetPosition.armAngle(),
                                        targetPosition.wristAngle());
                                HotLogger.Log("AA debug Arm", 7);
                                System.out.println("LogStuff: 7");
                            }
                            else
                            {
                                setTargetSafeElevator(targetPosition);
                                HotLogger.Log("AA debug Arm", 8);
                                System.out.println("LogStuff: 8");
                            }
                        }
                    }
                }
                // Switching sides
                else
                {
                    // Swapping from front to back
                    if (getArmSide(targetPosition.armAngle()) == RobotSide.BACK)
                    {
                        // We are in arm position for the other side
                        if (arm.getPosition() - ARM_TOLERANCE < ManipulatorSetPoint.limit_front_high.armAngle())
                        {
                            // Fix wrist
                            if (wrist.getPosition() - WRIST_TOLERANCE > ManipulatorSetPoint.limit_front_high
                                    .wristAngle())
                            {
                                setTargetSafeElevator(OVER_THE_TOP_HEIGHT, prevArmAngle,
                                        ManipulatorSetPoint.limit_front_high.wristAngle());
                            }
                            // Go to far limit
                            else if (targetPosition.armAngle() < ManipulatorSetPoint.limit_back_high.armAngle())
                            {
                                setTargetSafeElevator(OVER_THE_TOP_HEIGHT,
                                        ManipulatorSetPoint.limit_back_high.armAngle(),
                                        ManipulatorSetPoint.limit_back_high.wristAngle());
                            }

                            // Go to position
                            else
                            {
                                setTargetSafeElevator(OVER_THE_TOP_HEIGHT, targetPosition.armAngle(),
                                        targetPosition.wristAngle());
                            }
                        }
                        // Goto limit
                        else if (ManipulatorSetPoint.limit_front_low.wristAngle() < wrist.getPosition()
                                - WRIST_TOLERANCE
                                || arm.getPosition() - ARM_TOLERANCE > ManipulatorSetPoint.limit_front_low.armAngle())
                        {
                            // Move the wrist first, should always be safe as we are on the same side
                            setTargetSafeElevator(OVER_THE_TOP_HEIGHT, ManipulatorSetPoint.limit_front_low.armAngle(),
                                    ManipulatorSetPoint.limit_front_low.wristAngle());
                        }
                        // Goto limit
                        else if (ManipulatorSetPoint.limit_front_high.wristAngle() < wrist.getPosition()
                                - WRIST_TOLERANCE
                                || arm.getPosition() - ARM_TOLERANCE > ManipulatorSetPoint.limit_front_high.armAngle()
                                || elevator.getPosition() - ELEVATOR_TOLERANCE < OVER_THE_TOP_HEIGHT)
                        {
                            // Move the wrist first, should always be safe as we are on the same side
                            setTargetSafeElevator(OVER_THE_TOP_HEIGHT, ManipulatorSetPoint.limit_front_high.armAngle(),
                                    ManipulatorSetPoint.limit_front_high.wristAngle());
                        }
                        else
                        {
                            setTargetSafeElevator(OVER_THE_TOP_HEIGHT, ManipulatorSetPoint.limit_back_high.armAngle(),
                                    ManipulatorSetPoint.limit_back_high.wristAngle());
                        }

                    }
                    // Swapping from back to front
                    else if (getArmSide(targetPosition.armAngle()) == RobotSide.FRONT)
                    {
                        // We are in arm position for the other side
                        if (arm.getPosition() + ARM_TOLERANCE > ManipulatorSetPoint.limit_back_high.armAngle())
                        {
                            // Fix wrist
                            if (wrist.getPosition() + WRIST_TOLERANCE < ManipulatorSetPoint.limit_back_high
                                    .wristAngle())
                                setTargetSafeElevator(OVER_THE_TOP_HEIGHT, prevArmAngle,
                                        ManipulatorSetPoint.limit_back_high.wristAngle());
                            // Go to far limit
                            else if (targetPosition.armAngle() > ManipulatorSetPoint.limit_front_high.armAngle())
                                setTargetSafeElevator(OVER_THE_TOP_HEIGHT,
                                        ManipulatorSetPoint.limit_front_high.armAngle(),
                                        ManipulatorSetPoint.limit_front_high.wristAngle());
                            // Go to position
                            else
                                setTargetSafeElevator(OVER_THE_TOP_HEIGHT, targetPosition.armAngle(),
                                        targetPosition.wristAngle());
                        }
                        // Goto limit
                        else if (ManipulatorSetPoint.limit_back_low.wristAngle() > wrist.getPosition() + WRIST_TOLERANCE
                                || arm.getPosition() + ARM_TOLERANCE < ManipulatorSetPoint.limit_back_low.armAngle())
                        {
                            // Move the wrist first, should always be safe as we are on the same side
                            setTargetSafeElevator(OVER_THE_TOP_HEIGHT, ManipulatorSetPoint.limit_back_low.armAngle(),
                                    ManipulatorSetPoint.limit_back_low.wristAngle());
                        }
                        // Goto limit
                        else if (ManipulatorSetPoint.limit_back_high.wristAngle() > wrist.getPosition()
                                + WRIST_TOLERANCE
                                || arm.getPosition() + ARM_TOLERANCE < ManipulatorSetPoint.limit_back_high.armAngle()
                                || elevator.getPosition() - ELEVATOR_TOLERANCE < OVER_THE_TOP_HEIGHT)
                        {
                            // Move the wrist first, should always be safe as we are on the same side
                            setTargetSafeElevator(OVER_THE_TOP_HEIGHT, ManipulatorSetPoint.limit_back_high.armAngle(),
                                    ManipulatorSetPoint.limit_back_high.wristAngle());
                        }
                        else
                        {
                            setTargetSafeElevator(OVER_THE_TOP_HEIGHT, ManipulatorSetPoint.limit_front_high.armAngle(),
                                    ManipulatorSetPoint.limit_front_high.wristAngle());
                        }
                    }
                }
            }
        }
        prevArmAngle = arm.getPosition();
        prevWristAngle = wrist.getPosition();

    }

    private void setTargets(double elevTarget, double armTarget, double wristTarget)
    {

        setTargets(elevTarget, armTarget, wristTarget, ManipulatorSetPoint.firstPosition.frontFlipper(),
                ManipulatorSetPoint.firstPosition.backFlipper());
    }

    boolean holdingElevator = false;
    boolean holdingArm = false;
    boolean holdingWrist = false;

    private void setTargets(double elevTarget, double armTarget, double wristTarget, double frontFlipperTarget,
            double backFlipperTarget)
    {
        holdingElevator = (elevTarget == prevElevHeight);
        holdingArm = (armTarget == prevArmAngle);
        holdingWrist = (wristTarget == prevWristAngle);

        elevator.setTarget(elevTarget);
        arm.setTarget(armTarget);
        wrist.setTarget(wristTarget);
        SmartDashboard.putNumber("elevatorTarget", elevTarget);
        SmartDashboard.putNumber("armTarget", armTarget);
        SmartDashboard.putNumber("wristTarget", wristTarget);
        // frontFlipper.setTarget(frontFlipperTarget);
        // backFlipper.setTarget(backFlipperTarget);
    }

    private void setTargets(IManipulatorSetPoint elevTarget, IManipulatorSetPoint armTarget,
            IManipulatorSetPoint wristTarget)
    {
        SmartDashboard.putNumber("elevatorTarget", elevTarget.elevatorHeight());
        SmartDashboard.putNumber("armTarget", armTarget.armAngle());
        SmartDashboard.putNumber("wristTarget", wristTarget.wristAngle());

        setTargets(elevTarget.elevatorHeight(), armTarget.armAngle(), wristTarget.wristAngle());

    }

    // This is the side the arm is on
    private RobotSide getArmSide(double armAngle)
    {
        if (armAngle > -5.0)
            return RobotSide.FRONT;
        return RobotSide.BACK;
    }

    private boolean isArmSafe()
    {
        // May have to tweak a tolerance
        return (arm.getPosition() > ManipulatorSetPoint.limit_back_high.armAngle()
                && arm.getPosition() < ManipulatorSetPoint.limit_front_high.armAngle());
    }

    private boolean isWristSafe()
    {
        // May have to tweak a tolerance
        return (wrist.getPosition() > ManipulatorSetPoint.limit_back_high.wristAngle()
                && wrist.getPosition() < ManipulatorSetPoint.limit_front_high.wristAngle());
    }

    private boolean willCollideWithFrame(double elevHeight, double armAngle, double wristAngle)
    {
        if ((arm.getPosition() > 90.0 && widthManipulator(armAngle, wristAngle) < FRAME_LENGTH)
                || (arm.getPosition() < -90.0 && widthManipulator(armAngle, wristAngle) > -FRAME_LENGTH))
            return ((heightManipulator(elevHeight, arm.getPosition(), wrist.getPosition())
                    + 13.5 / 2 * Math.sin(Math.toRadians(wrist.getPosition())) < FRAME_COLLISION)
                    || (heightManipulator(elevHeight, arm.getPosition(), wrist.getPosition())
                            - 13.5 / 2 * Math.sin(Math.toRadians(wrist.getPosition())) < FRAME_COLLISION));
        return false;
    }

    private boolean willCollideWithSupports(double elevHeight, double armAngle, double wristAngle)
    {

        double xTop = widthManipulator(armAngle, wristAngle) - 13.5 / 2 * Math.cos(Math.toRadians(wristAngle));
        double yTop = heightManipulator(elevHeight, armAngle, wristAngle)
                + 13.5 / 2 * Math.sin(Math.toRadians(wristAngle));

        double xBottom = widthManipulator(armAngle, wristAngle) + 13.5 / 2 * Math.cos(Math.toRadians(wristAngle));
        double yBottom = heightManipulator(elevHeight, armAngle, wristAngle)
                - 13.5 / 2 * Math.sin(Math.toRadians(wristAngle));

        SmartDashboard.putNumber("xTop", xTop);
        SmartDashboard.putNumber("yTop", yTop);
        SmartDashboard.putNumber("xBottom", xBottom);
        SmartDashboard.putNumber("yBottom", yBottom);

        if ((yBottom < 44 && Math.abs(xBottom) < 5) || (yTop < 44 && Math.abs(xTop) < 5))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    private double lengthWristY(double angle)
    {
        // in case we need to adjust this
        return WRIST_LENGTH * Math.cos(Math.toRadians(angle));
    }

    private double lengthWristX(double angle)
    {
        // in case we need to adjust this again
        return WRIST_LENGTH * Math.sin(Math.toRadians(angle));
    }

    private double heightManipulator(double elvatorHeight, double armAngle, double wristAngle)
    {
        return elvatorHeight + ELEVATOR_LENGTH + lengthWristY(wristAngle)
                + ARM_LENGTH * Math.cos(Math.toRadians(armAngle));
    }

    private double widthManipulator(double armAngle, double wristAngle)
    {
        return ARM_LENGTH * Math.sin(Math.toRadians(armAngle)) + lengthWristX(wristAngle);
    }

    public void Update()
    {
        arm.checkEncoder();
        wrist.checkEncoder();
        elevator.checkEncoder(0);
        intake.Update();
        pneumaticIntake.Update();
        ManipulatorSetPoint frontTargetPosition = null;
        ManipulatorSetPoint backTargetPosition = null;

        boolean isLeftTriggerPressed = (operator.getLeftTrigger() >= .25);
        boolean isRightTriggerPressed = (operator.getRightTrigger() >= .25);

        if (operator.getButtonX())
        {
            frontTargetPosition = ManipulatorSetPoint.carry_front;
            backTargetPosition = ManipulatorSetPoint.carry_back;
        }
        else if (operator.getButtonA())
        {
            frontTargetPosition = ManipulatorSetPoint.hatch_low_front;
            backTargetPosition = ManipulatorSetPoint.hatch_low_back;
        }
        else if (operator.getButtonB())
        {
            frontTargetPosition = ManipulatorSetPoint.hatch_mid_front;
            backTargetPosition = ManipulatorSetPoint.hatch_mid_back;
        }
        else if (operator.getButtonY())
        {
            frontTargetPosition = ManipulatorSetPoint.hatch_high_front;
            backTargetPosition = ManipulatorSetPoint.hatch_high_back;
        }
        else if (operator.getButtonLeftBumper())
        {
            /*
             * frontTargetPosition = ManipulatorSetPoint.cargo_rocketLow_front;
             * backTargetPosition = ManipulatorSetPoint.cargo_rocketLow_back;
             */
        }
        else if (operator.getButtonRightBumper())
        {
            /*
             * frontTargetPosition = ManipulatorSetPoint.cargo_shuttle_front;
             * backTargetPosition = ManipulatorSetPoint.cargo_shuttle_back;
             */
        }
        else if (isLeftTriggerPressed == true)
        {
            /*
             * frontTargetPosition = ManipulatorSetPoint.cargo_rocketMid_front;
             * backTargetPosition = ManipulatorSetPoint.cargo_rocketMid_back;
             */
        }

        else if (isRightTriggerPressed == true)
        {
            /*
             * frontTargetPosition = ManipulatorSetPoint.cargo_rocketHigh_front;
             * backTargetPosition = ManipulatorSetPoint.cargo_rocketHigh_back;
             */
        }
        else if (operator.getButtonLeftStick())
        {
            /*
             * frontTargetPosition = ManipulatorSetPoint.cargo_pickup_front;
             * backTargetPosition = ManipulatorSetPoint.cargo_pickup_back;
             */
        }
        else if (operator.getButtonRightStick())
        {
            /*
             * frontTargetPosition = ManipulatorSetPoint.cargo_pickup_front;
             * backTargetPosition = ManipulatorSetPoint.cargo_pickup_back;
             */
        }

        boolean score = false;
        score = operator.getButtonBack();
        score = driver.getButtonA();
        score = operator.getButtonBack();

        if (operator.getButtonStart() && !startButtonPrevious)
        {
            commandToBack = !commandToBack;
        }

        // elevator.setTarget(ManipulatorSetPoint.hatch_low_front.elevatorHeight());
        if (frontTargetPosition != null || backTargetPosition != null)
        {
            IManipulatorSetPoint setPoint = (commandToBack) ? backTargetPosition : frontTargetPosition;
            if (score)
            {
                setPoint = CreateScoreSetPoint(setPoint);
            }
            Control(setPoint);
            SmartDashboard.putBoolean("Disabled thing", false);
        }
        else
        {
            elevator.disable();
            arm.disable();
            wrist.disable();

            // elevator.manual(operator.getStickLY());
            arm.manual(operator.getStickRY());
            /*
             * if (commandToBack) backFlipper.manual(operator.getStickRY()); else
             * frontFlipper.manual(operator.getStickRY());
             */

            SmartDashboard.putBoolean("Disabled thing", true);
        }
        // elevator.setTarget(10);

        SmartDashboard.putBoolean("COLLIDE FRAME",
                willCollideWithFrame(elevator.getPosition(), arm.getPosition(), wrist.getPosition()));
        SmartDashboard.putBoolean("COLLIDE SUPPORTS",
                willCollideWithSupports(elevator.getPosition(), arm.getPosition(), wrist.getPosition()));
        SmartDashboard.putNumber("Wrist Width", widthManipulator(arm.getPosition(), wrist.getPosition()));
        SmartDashboard.putNumber("Wrist Height",
                heightManipulator(elevator.getPosition(), arm.getPosition(), wrist.getPosition()));

        SmartDashboard.putNumber("WristTopPostitionX", widthManipulator(arm.getPosition(), wrist.getPosition())
                - 13.5 / 2 * Math.cos(Math.toRadians(wrist.getPosition())));
        SmartDashboard.putNumber("WristBottonPostitionX", widthManipulator(arm.getPosition(), wrist.getPosition())
                + 13.5 / 2 * Math.cos(Math.toRadians(wrist.getPosition())));

        SmartDashboard.putNumber("WristTopPostitionY",
                heightManipulator(elevator.getPosition(), arm.getPosition(), wrist.getPosition())
                        + 13.5 / 2 * Math.sin(Math.toRadians(wrist.getPosition())));
        SmartDashboard.putNumber("WristBottonPostitionY",
                heightManipulator(elevator.getPosition(), arm.getPosition(), wrist.getPosition())
                        - 13.5 / 2 * Math.sin(Math.toRadians(wrist.getPosition())));

        startButtonPrevious = operator.getButtonStart();
        backButtonPrevious = operator.getButtonBack();

        if (!holdingElevator)
            prevElevHeight = elevator.getPosition();
        if (!holdingArm)
            prevArmAngle = arm.getPosition();
        if (!holdingWrist)
            prevWristAngle = wrist.getPosition();
    }

    private IManipulatorSetPoint CreateScoreSetPoint(IManipulatorSetPoint setPoint)
    {
        double armAngle = Math.toRadians(setPoint.armAngle());

        double newArmX = Math.sin(armAngle) * ARM_LENGTH;
        double newArmAngle;

        SmartDashboard.putNumber("New Arm X", newArmX);

        if (getArmSide(setPoint.armAngle()) == RobotSide.FRONT)
        {
            newArmX += SCORE_DISTANCE;
            if (newArmX > ARM_LENGTH)
                newArmX = ARM_LENGTH;

            // Inverse sin on -PI/2 < X < PI/2
            newArmAngle = Math.asin(newArmX / ARM_LENGTH);

            // For two solutions, pick the angle that is closer to the initial setpoint
            if (armAngle > (Math.PI / 2.0))
                newArmAngle = Math.PI - newArmAngle;
        }
        else
        {
            newArmX -= SCORE_DISTANCE;
            if (newArmX < -ARM_LENGTH)
                newArmX = -ARM_LENGTH;

            // Inverse sin on -PI/2 < X < PI/2
            newArmAngle = Math.asin(newArmX / ARM_LENGTH);

            // For two solutions, pick the angle that is closer to the initial setpoint
            if (armAngle < -(Math.PI / 2.0))
                newArmAngle = -Math.PI - newArmAngle;
        }

        double newElevatorHeight = setPoint.elevatorHeight()
                + (ARM_LENGTH * (Math.cos(armAngle) - Math.cos(newArmAngle)));

        SmartDashboard.putNumber("New Arm Angle", Math.toDegrees(newArmAngle));
        SmartDashboard.putNumber("New Elevator Height", newElevatorHeight);

        return new ManualManipulatorSetPoint(Math.toDegrees(newArmAngle), setPoint.wristAngle(), newElevatorHeight,
                setPoint.frontFlipper(), setPoint.backFlipper());
    }
}