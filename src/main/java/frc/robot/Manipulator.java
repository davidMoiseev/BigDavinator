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
import frc.robot.constants.FlipperConstants;
import frc.robot.constants.IManipulatorSetPoint;
import frc.robot.constants.ManipulatorSetPoint;
import frc.robot.constants.ManualManipulatorSetPoint;
import frc.robot.constants.IRobotCommandProvider;

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

    public static final List<String> LoggerTags = new ArrayList<>(
            Arrays.asList("AA debug Arm", "armTarget", "wristTarget", "elevatorTarget"));

    private Elevator elevator;
    private Intake intake;
    private IntakePneumatics pneumaticIntake;
    private Wrist wrist;
    private Arm arm;
    private FlipperActuator frontFlipper;
    private FlipperActuator backFlipper;
    private Solenoid climber;

    private HotController driver;
    private HotController operator;
    private ArmPigeon armPigeon;
    private DriveTrain drivetrain;
    private InitializationState initializationState;
    private ManipulatorState manipulatorState;

    private double prevElevHeight;
    private double prevArmAngle;
    private double prevWristAngle;
    private double prevFrontFlipperAngle;
    private double prevBackFlipperAngle;

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

    private final double FLIPPER_LENGTH = 13;
    private final double FLIPPER_HEIGHT = 3;

    public Manipulator(HotController operator, HotController driver, TalonSRX rightElevator, TalonSRX intake,
            DriveTrain drivetrain)
    {
        this.elevator = new Elevator(new TalonSRX(WiringIDs.LEFT_ELEVATOR), rightElevator);
        this.wrist = new Wrist(WiringIDs.WRIST);
        this.arm = new Arm(WiringIDs.SHOULDER);
        this.armPigeon = new ArmPigeon(WiringIDs.PIGEON_ARM);
        this.intake = new Intake(driver);
        this.pneumaticIntake = new IntakePneumatics(driver);
        this.frontFlipper = new FlipperActuator(WiringIDs.FLIPPER_FRONT);
        this.backFlipper = new FlipperActuator(WiringIDs.FLIPPER_BACK);

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

            // frontFlipper.setTarget(FlipperConstants.CARRY_FRONT);
            // backFlipper.setTarget(FlipperConstants.CARRY_BACK);
            if (frontFlipper.reachedTarget() && backFlipper.reachedTarget())
            {
                elevator.setTarget(ManipulatorSetPoint.firstPosition);
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
        }

        if (manipulatorState == ManipulatorState.atTarget)
        {
            // If it isn't
            // else
            {
                // Same side
                if (getArmSide(targetPosition.armAngle()) == getArmSide(arm.getPosition()))
                {
                    // Front side
                    if (getArmSide(targetPosition.armAngle()) == RobotSide.FRONT)
                    {
                        // If arm is in the range where it won't collide with the frame or elevator
                        // supports
                        if (arm.getPosition() + ARM_TOLERANCE < ManipulatorSetPoint.limit_front_extra_low.armAngle()
                                && targetPosition.armAngle() > ManipulatorSetPoint.limit_front_extra_low.armAngle())
                        {
                            HotLogger.Log("AA debug Arm", 6);
                            System.out.println("LogStuff: 6");
                            setTargets(targetPosition.elevatorHeight(),
                                    ManipulatorSetPoint.limit_front_extra_low.armAngle(),
                                    ManipulatorSetPoint.limit_front_extra_low.wristAngle(),
                                    targetPosition.frontFlipper(), targetPosition.backFlipper());
                        }
                        else
                        {
                            setTargets(targetPosition, targetPosition, targetPosition, targetPosition, targetPosition);

                        }
                    }
                    // Back side
                    else
                    {
                        // If arm is in the range where it won't collide with the frame or elevator
                        // supports
                        if (arm.getPosition() - ARM_TOLERANCE > ManipulatorSetPoint.limit_back_extra_low.armAngle()
                                && targetPosition.armAngle() < ManipulatorSetPoint.limit_back_extra_low.armAngle())
                        {
                            HotLogger.Log("AA debug Arm", 7);
                            System.out.println("LogStuff: 7");
                            setTargets(targetPosition.elevatorHeight(),
                                    ManipulatorSetPoint.limit_back_extra_low.armAngle(),
                                    ManipulatorSetPoint.limit_back_extra_low.wristAngle(),
                                    targetPosition.frontFlipper(), targetPosition.backFlipper());
                        }
                        else
                        {
                            setTargets(targetPosition, targetPosition, targetPosition, targetPosition, targetPosition);
                        }
                    }
                }
                // Switching sides
                else
                {
                    double safeHeight = (elevator.getPosition() - ELEVATOR_TOLERANCE > OVER_THE_TOP_HEIGHT)
                            ? prevElevHeight
                            : OVER_THE_TOP_HEIGHT;
                    safeHeight = (targetPosition.elevatorHeight() > OVER_THE_TOP_HEIGHT)
                            ? targetPosition.elevatorHeight()
                            : safeHeight;

                    // Swapping from front to back
                    if (getArmSide(targetPosition.armAngle()) == RobotSide.BACK)
                    {
                        // If angle is farther than near target, make it the target
                        double farWristAngle = (targetPosition.wristAngle() < ManipulatorSetPoint.limit_front_high
                                .wristAngle()) ? targetPosition.wristAngle()
                                        : ManipulatorSetPoint.limit_front_high.wristAngle();
                        if (farWristAngle < ManipulatorSetPoint.limit_back_high.wristAngle())
                            farWristAngle = ManipulatorSetPoint.limit_back_high.wristAngle();

                        // Arm still coming up
                        if (arm.getPosition() > ManipulatorSetPoint.limit_front_high.armAngle())
                        {
                            setTargets(safeHeight, targetPosition.armAngle(), farWristAngle,
                                    targetPosition.frontFlipper(), targetPosition.backFlipper());
                        }
                        // Arm is up but wrist is not ready
                        // wrist is not within limits to go over, stop arm
                        else if (wrist.getPosition() - WRIST_TOLERANCE > ManipulatorSetPoint.limit_front_high
                                .wristAngle()
                                || wrist.getPosition() + WRIST_TOLERANCE < ManipulatorSetPoint.limit_back_high
                                        .wristAngle()
                                        && !(elevator.getPosition() + ELEVATOR_TOLERANCE > ELEVATOR_CLEAR_HEIGHT))
                        {
                            setTargets(safeHeight, prevArmAngle, farWristAngle, targetPosition.frontFlipper(),
                                    targetPosition.backFlipper());
                        }
                        // Arm and wrist are ready, but elevator is not
                        else if (elevator.getPosition() + ELEVATOR_TOLERANCE < safeHeight)
                        {
                            setTargets(safeHeight, prevArmAngle, prevWristAngle, targetPosition.frontFlipper(),
                                    targetPosition.backFlipper());
                        }
                        // Over the top
                        else
                        {
                            setTargets(safeHeight, targetPosition.armAngle(), farWristAngle,
                                    targetPosition.frontFlipper(), targetPosition.backFlipper());
                        }
                    }
                    // Swapping from back to front
                    else if (getArmSide(targetPosition.armAngle()) == RobotSide.FRONT)
                    {
                        // If angle is closer than far elevator side, make it the target
                        double farWristAngle = (targetPosition.wristAngle() > ManipulatorSetPoint.limit_back_high
                                .wristAngle()) ? targetPosition.wristAngle()
                                        : ManipulatorSetPoint.limit_back_high.wristAngle();
                        // If angle is too far, goto far side
                        if (farWristAngle > ManipulatorSetPoint.limit_front_high.wristAngle())
                            farWristAngle = ManipulatorSetPoint.limit_front_high.wristAngle();

                        // Arm still coming up
                        if (arm.getPosition() < ManipulatorSetPoint.limit_back_high.armAngle())
                        {
                            setTargets(safeHeight, targetPosition.armAngle(), farWristAngle,
                                    targetPosition.frontFlipper(), targetPosition.backFlipper());
                        }
                        // Arm is up but wrist is not ready
                        // wrist is not within limits to go over, stop arm
                        else if (wrist.getPosition() - WRIST_TOLERANCE > ManipulatorSetPoint.limit_front_high
                                .wristAngle()
                                || wrist.getPosition() + WRIST_TOLERANCE < ManipulatorSetPoint.limit_back_high
                                        .wristAngle()
                                        && !(elevator.getPosition() + ELEVATOR_TOLERANCE > ELEVATOR_CLEAR_HEIGHT))
                        {
                            setTargets(safeHeight, prevArmAngle, farWristAngle, targetPosition.frontFlipper(),
                                    targetPosition.backFlipper());
                        }
                        // Arm and wrist are ready, but elevator is not
                        else if (elevator.getPosition() + ELEVATOR_TOLERANCE < safeHeight)
                        {
                            setTargets(safeHeight, prevArmAngle, prevWristAngle, targetPosition.frontFlipper(),
                                    targetPosition.backFlipper());
                        }
                        // Over the top
                        else
                        {
                            setTargets(safeHeight, targetPosition.armAngle(), farWristAngle,
                                    targetPosition.frontFlipper(), targetPosition.backFlipper());
                        }
                    }
                }
            }
        }
        prevArmAngle = arm.getPosition();
        prevWristAngle = wrist.getPosition();

    }

    boolean holdingElevator = false;
    boolean holdingArm = false;
    boolean holdingWrist = false;
    boolean holdingFrontFlipper = false;
    boolean holdingBackFlipper = false;

    private boolean flipperOnTarget(FlipperActuator f, double target)
    {
        return ((f.getPosition() - 15 < target) && (f.getPosition() + 15 > target));
    }

    private double getArmY(double arm, double elevator)
    {
        return Math.cos(Math.toRadians(arm)) * ARM_LENGTH + ELEVATOR_LENGTH + elevator;
    }

    private double getWristY(double arm, double wrist, double elevator)
    {
        double wristY = Math.cos(Math.toRadians(wrist)) * WRIST_LENGTH;
        return wristY + getArmY(arm, elevator);
    }

    private double getArmX(double arm)
    {
        return Math.abs(Math.sin(Math.toRadians(arm)) * ARM_LENGTH);
    }

    private double getWristX(double arm, double wrist)
    {
        double wristX = Math.abs(Math.sin(Math.toRadians(wrist)) * WRIST_LENGTH);
        return wristX + getArmX(arm);
    }

    private double getFlipperX(double flipper)
    {
        return FLIPPER_LENGTH * Math.abs(Math.sin(Math.toRadians(flipper)));
    }

    private double getFlipperY(double flipper)
    {
        return FLIPPER_LENGTH * Math.abs(Math.cos(Math.toRadians(flipper)));
    }

    private boolean flipperSafe(double wristX, double wristY, double armY, double flipperX, double flipperY)
    {
        return (wristX > flipperX && armY > flipperY) || wristY > flipperY;
    }

    private void setTargets(double elevTarget, double armTarget, double wristTarget, double frontFlipperTarget,
            double backFlipperTarget)
    {
        // The angle from arm angle, negative is counter-clockwise
        double wristAngleRelative = wrist.getPosition() - arm.getPosition();
        SmartDashboard.putNumber("wristAngleRelative", wristAngleRelative);

        // Elevator waits if the arm is coming down
        if (elevTarget < elevator.getPosition() - ELEVATOR_TOLERANCE && Math.abs(arm.getPosition()) > 90
                && Math.abs(armTarget - arm.getPosition()) > ARM_TOLERANCE)
        {
            elevTarget = prevElevHeight;
        }

        double wristY = getWristY(arm.getPosition(), wrist.getPosition(), elevator.getPosition());
        double armY = getArmY(arm.getPosition(), elevator.getPosition());
        double targetWristY = getWristY(armTarget, wristTarget, elevTarget);

        double frontFlipperY = getFlipperY(frontFlipper.getPosition());
        double backFlipperY = getFlipperY(backFlipper.getPosition());
        double frontFlipperX = getFlipperX(frontFlipper.getPosition());
        double backFlipperX = getFlipperX(backFlipper.getPosition());
        double targetFlipperY = getFlipperY(frontFlipperTarget);
        double targetFlipperX = getFlipperX(frontFlipperTarget);
        SmartDashboard.putNumber("frontFlipperY", frontFlipperY);
        SmartDashboard.putNumber("backFlipperY", backFlipperY);
        SmartDashboard.putNumber("frontFlipperX", frontFlipperX);
        SmartDashboard.putNumber("backFlipperX", backFlipperX);
        SmartDashboard.putNumber("targetFlipperY", targetFlipperY);
        SmartDashboard.putNumber("targetFlipperX", targetFlipperX);

        double flipperMaxY = 25;
        double flipperMaxX = 16;

        double wristX = getWristX(arm.getPosition(), wrist.getPosition());

        boolean flipperSafe = false;

        if (wristX > flipperMaxX && armY > flipperMaxY)
            flipperSafe = true;
        if (wristY > flipperMaxY)
            flipperSafe = true;
        // Flipper going out and up/down but not passing 90
        if (frontFlipperTarget > frontFlipper.getPosition() && frontFlipperY < wristY && targetFlipperY < wristY
                && !(frontFlipperTarget > 90 && frontFlipper.getPosition() < 90))
            flipperSafe = true;
        if (frontFlipperTarget < frontFlipper.getPosition() && frontFlipperY < wristY && targetFlipperY < wristY
                && !(frontFlipperTarget < 90 && frontFlipper.getPosition() > 90))
            flipperSafe = true;

        SmartDashboard.putNumber("wristX", wristX);
        SmartDashboard.putNumber("wristY", wristY);
        SmartDashboard.putNumber("armY", armY);

        SmartDashboard.putBoolean("flipperSafe", flipperSafe);

        // Flipper waiting
        if (getArmSide(arm.getPosition()) == RobotSide.FRONT && !flipperOnTarget(frontFlipper, frontFlipperTarget))
        {
            if (targetWristY > flipperMaxY && wristY < flipperMaxY)
                frontFlipperTarget = prevFrontFlipperAngle;
            else if (targetWristY < flipperMaxY && wristY > flipperMaxY && arm.getPosition() > 100)
            {
                armTarget = prevArmAngle;
                wristTarget = prevWristAngle;
                elevTarget = prevElevHeight;
            }
            else if (targetWristY < flipperMaxY && wristY < flipperMaxY)
            {
                armTarget = 90;
                wristTarget = 90;
                elevTarget = 20;
                frontFlipperTarget = prevFrontFlipperAngle;
            }
        }
        else if (getArmSide(arm.getPosition()) == RobotSide.BACK && !flipperOnTarget(backFlipper, backFlipperTarget))
        {
            if (targetWristY > flipperMaxY && wristY < flipperMaxY)
                backFlipperTarget = prevBackFlipperAngle;
            else if (targetWristY < flipperMaxY && wristY > flipperMaxY && arm.getPosition() < -100)
            {
                armTarget = prevArmAngle;
                wristTarget = prevWristAngle;
                elevTarget = prevElevHeight;
            }
            else if (targetWristY < flipperMaxY && wristY < flipperMaxY)
            {
                armTarget = -90;
                wristTarget = -90;
                elevTarget = 20;
                backFlipperTarget = prevBackFlipperAngle;
            }
        }

        if (arm.getPosition() + ARM_TOLERANCE < armTarget && wristAngleRelative < -110)
        {
            armTarget = prevArmAngle;
        }
        // arm Clockwise wrist clockwise
        else if (arm.getPosition() + ARM_TOLERANCE < armTarget && wristAngleRelative > 110)
        {
            wristTarget = prevWristAngle;
        }
        // arm counterclockwise wrist counterclockwise
        else if (arm.getPosition() - ARM_TOLERANCE > armTarget && wristAngleRelative < -110)
        {
            wristTarget = prevWristAngle;
        }
        // arm clockwise wrist clockwise
        else if (arm.getPosition() - ARM_TOLERANCE > armTarget && wristAngleRelative > 110)
        {
            armTarget = prevArmAngle;
        }

        // frontFlipper.setTarget(frontFlipperTarget);
        // backFlipper.setTarget(backFlipperTarget);

        holdingElevator = (elevTarget == prevElevHeight);
        holdingArm = (armTarget == prevArmAngle);
        holdingWrist = (wristTarget == prevWristAngle);
        holdingFrontFlipper = (frontFlipperTarget == prevFrontFlipperAngle);
        holdingBackFlipper = (backFlipperTarget == prevBackFlipperAngle);

        elevator.setTarget(elevTarget);
        arm.setTarget(armTarget);
        wrist.setTarget(wristTarget);

        // frontFlipper.control(frontFlipperTarget != 0)

        SmartDashboard.putNumber("elevatorTarget", elevTarget);
        HotLogger.Log("elevatorTarget", elevTarget);
        SmartDashboard.putNumber("armTarget", armTarget);
        HotLogger.Log("armTarget", armTarget);
        SmartDashboard.putNumber("wristTarget", wristTarget);
        HotLogger.Log("wristTarget", wristTarget);
        /*
         * SmartDashboard.putNumber("wristHolding", holdingWrist);
         * SmartDashboard.putNumber("armHolding", holdingArm);
         */
        SmartDashboard.putNumber("frontFlipperTarget", frontFlipperTarget);
        SmartDashboard.putNumber("backFlipperTarget", backFlipperTarget);
        SmartDashboard.putBoolean("frontFlipperOnTarget", flipperOnTarget(frontFlipper, frontFlipperTarget));
        SmartDashboard.putBoolean("backFlipperOnTarget", flipperOnTarget(backFlipper, backFlipperTarget));
        // frontFlipper.setTarget(frontFlipperTarget);
        // backFlipper.setTarget(backFlipperTarget);
    }

    private void setTargets(IManipulatorSetPoint elevTarget, IManipulatorSetPoint armTarget,
            IManipulatorSetPoint wristTarget, IManipulatorSetPoint frontFlipperTarget,
            IManipulatorSetPoint backFlipperTarget)
    {
        SmartDashboard.putNumber("elevatorTarget", elevTarget.elevatorHeight());
        SmartDashboard.putNumber("armTarget", armTarget.armAngle());
        SmartDashboard.putNumber("wristTarget", wristTarget.wristAngle());

        setTargets(elevTarget.elevatorHeight(), armTarget.armAngle(), wristTarget.wristAngle(),
                frontFlipperTarget.frontFlipper(), backFlipperTarget.backFlipper());

    }

    // This is the side the arm is on
    private RobotSide getArmSide(double armAngle)
    {
        if (armAngle > 0)
            return RobotSide.FRONT;
        return RobotSide.BACK;
    }

    public void Update(IRobotCommandProvider robotCommand)
    {
        arm.checkEncoder();
        wrist.checkEncoder();
        elevator.checkEncoder();
        intake.Update(robotCommand);
        pneumaticIntake.Update(robotCommand);

        double wristY = getWristY(arm.getPosition(), wrist.getPosition(), elevator.getPosition());

        SmartDashboard.putNumber("wristY", wristY);

        IManipulatorSetPoint setPoint = robotCommand.ManipulatorSetPoint();
        boolean score = robotCommand.ManipulatorScore();

        // To climb, we must be above climb height and targeting prep position
        drivetrain.setAllowClimberDeploy((setPoint == ManipulatorSetPoint.climb_prep)
                && elevator.getPosition() + ELEVATOR_TOLERANCE > ManipulatorSetPoint.climb_prep.elevatorHeight());

        // elevator.setTarget(ManipulatorSetPoint.hatch_low_front.elevatorHeight());
        if (setPoint != null)
        {
            if (score)
            {
                setPoint = CreateScoreSetPoint(setPoint);
            }
            if (robotCommand.HatchPickup())
            {
                double elevHeight = (setPoint.elevatorHeight() + 2.5 > 30 ? 30 : setPoint.elevatorHeight() + 2.5);
                setPoint = new ManualManipulatorSetPoint(setPoint.armAngle(), setPoint.wristAngle(), elevHeight,
                        setPoint.frontFlipper(), setPoint.backFlipper());
            }
            Control(setPoint);
            SmartDashboard.putBoolean("Disabled thing", false);
        }
        else
        {
            elevator.disable();
            arm.disable();
            wrist.disable();
            /*
             * frontFlipper.disable(); backFlipper.disable();
             */
            SmartDashboard.putBoolean("Disabled thing", true);
        }

        // frontFlipper.setTarget(FlipperConstants.CARRY_FRONT);

        SmartDashboard.putNumber("frontFlipper", frontFlipper.getPosition());
        SmartDashboard.putNumber("backFlipper", backFlipper.getPosition());
        SmartDashboard.putNumber("frontFlipper error", frontFlipper.getError());
        SmartDashboard.putNumber("backFlipper error", backFlipper.getError());

        if (!holdingElevator)
            prevElevHeight = elevator.getPosition();
        if (!holdingArm)
            prevArmAngle = arm.getPosition();
        if (!holdingWrist)
            prevWristAngle = wrist.getPosition();
        if (!holdingFrontFlipper)
            prevFrontFlipperAngle = frontFlipper.getPosition();
        if (!holdingBackFlipper)
            prevBackFlipperAngle = backFlipper.getPosition();
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
        newElevatorHeight = (newElevatorHeight > 33) ? 33 : newElevatorHeight;

        SmartDashboard.putNumber("New Arm Angle", Math.toDegrees(newArmAngle));
        SmartDashboard.putNumber("New Elevator Height", newElevatorHeight);

        return new ManualManipulatorSetPoint(Math.toDegrees(newArmAngle), setPoint.wristAngle(), newElevatorHeight,
                setPoint.frontFlipper(), setPoint.backFlipper());
    }
}