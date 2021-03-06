/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.manipulator;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.hotteam67.HotController;
import org.hotteam67.HotLogger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.WiringIDs;
import frc.robot.constants.WristConstants;
import frc.robot.manipulator.routines.HatchGrabber;
import frc.robot.manipulator.routines.HatchPlacer;
import frc.robot.manipulator.routines.HatchGrabber.HatchGrabberState;
import frc.robot.manipulator.routines.HatchPlacer.HatchPlacerState;
import frc.robot.RobotCommandProvider;
import frc.robot.RobotState;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.FlipperConstants;
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
        initializing, packagePosition, unPackaging, outOfPackage;
    }

    private enum RobotSide
    {
        FRONT, BACK
    }

    public static final List<String> LoggerTags = new ArrayList<>(
            Arrays.asList("leftLimit", "rightLimit", "armTarget", "wristTarget", "elevatorTarget", "frontFlipperTarget",
                    "backFlipperTarget", "elevator_collision", "flipper_collision", "arm_collision"));

    private final RobotState robotState;
    private final RobotState.Actions robotActionsState;

    private final Elevator elevator;
    private final Intake intake;
    private final IntakePneumatics intakePneumatics;
    private final Wrist wrist;
    private final Arm arm;
    private final Flipper frontFlipper;
    private final Flipper backFlipper;

    private final HatchPlacer hatchPlacer;
    private final HatchGrabber hatchGrabber;

    private final DigitalInput frontLeftLimit;
    private final DigitalInput frontRightLimit;
    private final DigitalInput backLeftLimit;
    private final DigitalInput backRightLimit;

    private final ArmPigeon armPigeon;
    private InitializationState initializationState;
    private ManipulatorState manipulatorState;

    private double prevElevHeight;
    private double prevArmAngle;
    private double prevWristAngle;
    private double prevFrontFlipperAngle;
    private double prevBackFlipperAngle;

    private boolean holdingElevator = false;
    private boolean holdingArm = false;
    private boolean holdingWrist = false;
    private boolean holdingFrontFlipper = false;
    private boolean holdingBackFlipper = false;

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

    public Manipulator(TalonSRX rightElevator, TalonSRX intake)
    {
        this.elevator = new Elevator(new TalonSRX(WiringIDs.LEFT_ELEVATOR), rightElevator);
        this.wrist = new Wrist(WiringIDs.WRIST);
        this.arm = new Arm(WiringIDs.SHOULDER);
        this.armPigeon = new ArmPigeon(WiringIDs.PIGEON_ARM);
        this.intake = new Intake();
        this.intakePneumatics = new IntakePneumatics();
        this.frontFlipper = new Flipper(WiringIDs.FLIPPER_FRONT, false, false);
        this.backFlipper = new Flipper(WiringIDs.FLIPPER_BACK, false, true);
        this.hatchPlacer = new HatchPlacer();
        this.hatchGrabber = new HatchGrabber();

        this.robotState = RobotState.getInstance();
        this.robotActionsState = RobotState.Actions.getInstance();

        frontLeftLimit = new DigitalInput(WiringIDs.FLIPPER_FRONT_LEFT_LIMIT_SWITCH);
        frontRightLimit = new DigitalInput(WiringIDs.FLIPPER_FRONT_RIGHT_LIMIT_SWITCH);
        backLeftLimit = new DigitalInput(WiringIDs.FLIPPER_BACK_LEFT_LIMIT_SWITCH);
        backRightLimit = new DigitalInput(WiringIDs.FLIPPER_BACK_RIGHT_LIMIT_SWITCH);

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
                SmartDashboard.putNumber("A Pigeon Angle", armPigeon.GetAngle());
                wrist.setPosition(armPigeon.GetAngle() - 134);
                elevator.zeroSensors();
                frontFlipper.initialize();
                backFlipper.initialize();
            }
        }

    }

    public void DisplaySensors()
    {
        SmartDashboard.putBoolean("Flipper Front Left Limit ", frontLeftLimit.get());
        SmartDashboard.putBoolean("Flipper Front Right Limit ", frontRightLimit.get());
        SmartDashboard.putBoolean("Flipper Back Left Limit ", backLeftLimit.get());
        SmartDashboard.putBoolean("Flipper Back Right Limit ", backRightLimit.get());
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
                manipulatorState = ManipulatorState.unPackaging;
            }
        }
        else if (manipulatorState == ManipulatorState.unPackaging)
        {
            double tmpArm = arm.getPosition();

            frontFlipper.control(targetPosition.frontFlipper());
            backFlipper.control(targetPosition.backFlipper());
            elevator.setTarget(ManipulatorSetPoint.firstPosition);
            if (elevator.getPosition() > ManipulatorSetPoint.firstPosition.elevatorHeight() - 2)
            {
                arm.setTarget(tmpArm);
                wrist.setTarget(ManipulatorSetPoint.firstPosition);
                if (wrist.getPosition() < 90)
                {
                    manipulatorState = ManipulatorState.outOfPackage;
                }
            }
        }

        if (manipulatorState == ManipulatorState.outOfPackage)
        {
            // If it isn't
            // else
            {
                // Same side
                if (getArmSide(targetPosition.armAngle()) == getArmSide(arm.getPosition())
                        || (elevator.getPosition() > 30 && Math.abs(arm.getPosition()) < 10))
                {
                    // Front side
                    if (getArmSide(targetPosition.armAngle()) == RobotSide.FRONT)
                    {
                        // If arm is in the range where it won't collide with the frame or elevator
                        // supports
                        if (arm.getPosition() + ARM_TOLERANCE < ManipulatorSetPoint.limit_front_extra_low.armAngle()
                                && targetPosition.armAngle() > ManipulatorSetPoint.limit_front_extra_low.armAngle())
                        {
                            setTargets(targetPosition.elevatorHeight(),
                                    ManipulatorSetPoint.limit_front_extra_low.armAngle(),
                                    ManipulatorSetPoint.limit_front_extra_low.wristAngle(),
                                    targetPosition.frontFlipper(), targetPosition.backFlipper());
                        }
                        else if (targetPosition.armAngle() > ManipulatorSetPoint.limit_front_high.armAngle()
                                && targetPosition.armAngle() < ManipulatorSetPoint.limit_front_low.armAngle()
                                || (!(willCollideWithFrame(elevator.getPosition(), arm.getPosition(),
                                        wrist.getPosition())
                                        || (willCollideWithSupports(elevator.getPosition(), arm.getPosition(),
                                                wrist.getPosition())))))
                        {
                            setTargets(targetPosition, targetPosition, targetPosition, targetPosition, targetPosition);

                        }
                        else
                        {
                            // setTargets(targetPosition, targetPosition, targetPosition);
                            setTargets(prevElevHeight, targetPosition.armAngle(), targetPosition.wristAngle(),
                                    targetPosition.frontFlipper(), targetPosition.backFlipper());
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
                            setTargets(targetPosition.elevatorHeight(),
                                    ManipulatorSetPoint.limit_back_extra_low.armAngle(),
                                    ManipulatorSetPoint.limit_back_extra_low.wristAngle(),
                                    targetPosition.frontFlipper(), targetPosition.backFlipper());
                        }
                        else if (targetPosition.armAngle() < ManipulatorSetPoint.limit_back_high.armAngle()
                                && targetPosition.armAngle() > ManipulatorSetPoint.limit_back_low.armAngle()
                                || !((willCollideWithFrame(elevator.getPosition(), arm.getPosition(),
                                        wrist.getPosition())
                                        || (willCollideWithSupports(elevator.getPosition(), arm.getPosition(),
                                                wrist.getPosition())))))
                        {
                            setTargets(targetPosition, targetPosition, targetPosition, targetPosition, targetPosition);
                        }
                        else
                        {
                            setTargets(prevElevHeight, targetPosition.armAngle(), targetPosition.wristAngle(),
                                    targetPosition.frontFlipper(), targetPosition.backFlipper());
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

    private boolean flipperOnTarget(Flipper f, double target)
    {
        return Math.abs(f.getPosition() - target) < 15
                || ((f.getPosition() > target && target > 90) || (f.getPosition() < target && target < 90));
    }

    private double getWristY(double arm, double wrist, double elevator)
    {
        double wristY = Math.cos(Math.toRadians(wrist)) * WRIST_LENGTH;
        double armY = Math.cos(Math.toRadians(arm)) * ARM_LENGTH;
        // SmartDashboard.putNumber("wrist component Y", wristY);
        // SmartDashboard.putNumber("arm component Y", armY);
        return wristY + ELEVATOR_LENGTH + elevator + armY;
    }

    private void setTargets(double elevTarget, double armTarget, double wristTarget, double frontFlipperTarget,
            double backFlipperTarget)
    {
        if (Math.abs(armTarget) > 165)
        {
            armTarget = 165 * Math.signum(armTarget);
        }

        if (frontFlipperTarget < FlipperConstants.CARRY_FRONT)
            frontFlipperTarget = FlipperConstants.CARRY_FRONT;
        else if (backFlipperTarget < FlipperConstants.CARRY_BACK)
            backFlipperTarget = FlipperConstants.CARRY_BACK;

        // The angle from arm angle, negative is counter-clockwise
        double wristAngleRelative = wrist.getPosition() - arm.getPosition();
        // SmartDashboard.putNumber("wristAngleRelative", wristAngleRelative);

        // Elevator waits if the arm is coming down
        if (elevTarget < elevator.getPosition() - ELEVATOR_TOLERANCE && Math.abs(arm.getPosition()) > 90
                && Math.abs(armTarget - arm.getPosition()) > ARM_TOLERANCE)
        {
            // SmartDashboard.putBoolean("elevator_collision", true);
            HotLogger.Log("elevator_collision", 1);
            elevTarget = prevElevHeight;
        }
        else
        {
            // SmartDashboard.putBoolean("elevator_collision", false);
            HotLogger.Log("elevator_collision", 0);
        }

        double wristY = getWristY(arm.getPosition(), wrist.getPosition(), elevator.getPosition());
        double targetWristY = getWristY(armTarget, wristTarget, elevTarget);

        // SmartDashboard.putNumber("wristY", wristY);
        double flipperY = 25;

        // Flipper waiting
        if (getArmSide(arm.getPosition()) == RobotSide.FRONT && !flipperOnTarget(frontFlipper, frontFlipperTarget))
        {
            if (targetWristY > flipperY && wristY < flipperY)
            {
                // SmartDashboard.putNumber("flipper_collision", 0);
                HotLogger.Log("flipper_collision", 0);
                frontFlipperTarget = prevFrontFlipperAngle;
            }
            else if (targetWristY < flipperY && wristY > flipperY && arm.getPosition() > 100)
            {
                // SmartDashboard.putNumber("flipper_collision", 1);
                HotLogger.Log("flipper_collision", 1);
                armTarget = prevArmAngle;
                wristTarget = prevWristAngle;
                elevTarget = prevElevHeight;
            }
            else if (targetWristY < flipperY && wristY < flipperY)
            {
                // SmartDashboard.putNumber("flipper_collision", 2);
                HotLogger.Log("flipper_collision", 2);
                armTarget = 90;
                wristTarget = 90;
                elevTarget = 20;
                frontFlipperTarget = prevFrontFlipperAngle;
            }
            else
            {
                // SmartDashboard.putNumber("flipper_collision", 6);
                HotLogger.Log("flipper_collision", 6);
            }

        }
        else if (getArmSide(arm.getPosition()) == RobotSide.BACK && !flipperOnTarget(backFlipper, backFlipperTarget))
        {
            if (targetWristY > flipperY && wristY < flipperY)
            {
                // SmartDashboard.putNumber("flipper_collision", 3);
                HotLogger.Log("flipper_collision", 3);
                backFlipperTarget = prevBackFlipperAngle;
            }
            else if (targetWristY < flipperY && wristY > flipperY && arm.getPosition() < -100)
            {
                // SmartDashboard.putNumber("flipper_collision", 4);
                HotLogger.Log("flipper_collision", 4);
                armTarget = prevArmAngle;
                wristTarget = prevWristAngle;
                elevTarget = prevElevHeight;
            }
            else if (targetWristY < flipperY && wristY < flipperY)
            {
                // SmartDashboard.putNumber("flipper_collision", 5);
                HotLogger.Log("flipper_collision", 5);
                armTarget = -90;
                wristTarget = -90;
                elevTarget = 20;
                backFlipperTarget = prevBackFlipperAngle;
            }
            else
            {
                // SmartDashboard.putNumber("flipper_collision", 7);
                HotLogger.Log("flipper_collision", 7);
            }
        }

        // Don't hit the flipper if we are on the way up
        if (arm.getPosition() > 100 && armTarget > arm.getPosition()
                && elevTarget > elevator.getPosition() + ELEVATOR_TOLERANCE && elevator.getPosition() < 7
                && frontFlipper.getPosition() < 100)
        {
            armTarget = prevArmAngle;
        }
        if (arm.getPosition() < -100 && armTarget < arm.getPosition()
                && elevTarget > elevator.getPosition() + ELEVATOR_TOLERANCE && elevator.getPosition() < 7
                && backFlipper.getPosition() < 100)
        {
            armTarget = prevArmAngle;
        }

        if (arm.getPosition() + ARM_TOLERANCE < armTarget && wristAngleRelative < -110)
        {
            armTarget = prevArmAngle;
            // SmartDashboard.putNumber("arm_collision", 0);
            HotLogger.Log("arm_collision", 0);
        }
        // arm Clockwise wrist clockwise
        else if (arm.getPosition() + ARM_TOLERANCE < armTarget && wristAngleRelative > 110)
        {
            wristTarget = prevWristAngle;
            // SmartDashboard.putNumber("arm_collision", 1);
            HotLogger.Log("arm_collision", 1);
        }
        // arm counterclockwise wrist counterclockwise
        else if (arm.getPosition() - ARM_TOLERANCE > armTarget && wristAngleRelative < -110)
        {
            wristTarget = prevWristAngle;
            // SmartDashboard.putNumber("arm_collision", 2);
            HotLogger.Log("arm_collision", 2);
        }
        // arm clockwise wrist clockwise
        else if (arm.getPosition() - ARM_TOLERANCE > armTarget && wristAngleRelative > 110)
        {
            armTarget = prevArmAngle;
            // SmartDashboard.putNumber("arm_collision", 3);
            HotLogger.Log("arm_collision", 3);
        }

        frontFlipper.control(frontFlipperTarget);
        backFlipper.control(backFlipperTarget);

        holdingElevator = (elevTarget == prevElevHeight);
        holdingArm = (armTarget == prevArmAngle);
        holdingWrist = (wristTarget == prevWristAngle);
        holdingFrontFlipper = (frontFlipperTarget == prevFrontFlipperAngle);
        holdingBackFlipper = (backFlipperTarget == prevBackFlipperAngle);

        elevator.setTarget(elevTarget);
        arm.setTarget(armTarget);

        if (!manualWrist)
            wrist.setTarget(wristTarget);

        IManipulatorSetPoint commandedSetPoint = new ManualManipulatorSetPoint(armTarget, wristTarget, elevTarget,
                frontFlipperTarget, backFlipperTarget);
        robotActionsState.setCommandedManipulatorSetPoint(commandedSetPoint);

        // frontFlipper.control(frontFlipperTarget != 0)

        // SmartDashboard.putNumber("elevatorTarget", elevTarget);
        HotLogger.Log("elevatorTarget", elevTarget);
        // SmartDashboard.putNumber("armTarget", armTarget);
        HotLogger.Log("armTarget", armTarget);
        // SmartDashboard.putNumber("wristTarget", wristTarget);
        HotLogger.Log("wristTarget", wristTarget);
        /*
         * SmartDashboard.putNumber("wristHolding", holdingWrist);
         * SmartDashboard.putNumber("armHolding", holdingArm);
         */
        // SmartDashboard.putNumber("frontFlipperTarget", frontFlipperTarget);
        HotLogger.Log("frontFlipperTarget", frontFlipperTarget);
        // SmartDashboard.putNumber("backFlipperTarget", backFlipperTarget);
        HotLogger.Log("backFlipperTarget", backFlipperTarget);
        // SmartDashboard.putBoolean("frontFlipperOnTarget",
        // flipperOnTarget(frontFlipper, frontFlipperTarget));
        // SmartDashboard.putBoolean("backFlipperOnTarget", flipperOnTarget(backFlipper,
        // backFlipperTarget));
        // frontFlipper.setTarget(frontFlipperTarget);
        // backFlipper.setTarget(backFlipperTarget);
    }

    public boolean onTarget(IManipulatorSetPoint setPoint)
    {
        return (Math.abs(setPoint.armAngle() - robotState.getArmPosition()) <= ArmConstants.allowableErrorDegrees
                && Math.abs(
                        setPoint.wristAngle() - robotState.getWristPosition()) <= WristConstants.allowableErrorDegrees
                && Math.abs(setPoint.elevatorHeight()
                        - robotState.getElevatorPosition()) <= ElevatorConstants.allowableErrorInches
                && flipperOnTarget(frontFlipper, setPoint.frontFlipper())
                && flipperOnTarget(backFlipper, setPoint.backFlipper()));
    }

    private void setTargets(IManipulatorSetPoint elevTarget, IManipulatorSetPoint armTarget,
            IManipulatorSetPoint wristTarget, IManipulatorSetPoint frontFlipperTarget,
            IManipulatorSetPoint backFlipperTarget)
    {
        // SmartDashboard.putNumber("elevatorTarget", elevTarget.elevatorHeight());
        // SmartDashboard.putNumber("armTarget", armTarget.armAngle());
        // SmartDashboard.putNumber("wristTarget", wristTarget.wristAngle());

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

        // SmartDashboard.putNumber("xTop", xTop);
        // SmartDashboard.putNumber("yTop", yTop);
        // SmartDashboard.putNumber("xBottom", xBottom);
        // SmartDashboard.putNumber("yBottom", yBottom);

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

    // Whether we are re-zeroing the arm
    boolean zeroingArm = false;

    boolean manualWrist = false;
    // Delay count for how long to center the hatc
    int hatchCenterTimer = 0;
    // Whether we are centering the hatch
    boolean hatchCenter = false;

    public void Update(RobotCommandProvider robotCommand)
    {
        Update(robotCommand, robotCommand.ManipulatorSetPoint());
    }

    public void Update(RobotCommandProvider robotCommand, ManipulatorSetPoint setPoint)
    {
        if (setPoint != null)
            RobotState.Actions.getInstance().setArmIsBack(getArmSide(setPoint.armAngle()) == RobotSide.BACK);
        armPigeon.CalibratePigeon();
        if (robotCommand.ARMREZERO() && !zeroingArm)
        {
            zeroingArm = true;
            armPigeon.GatherMeasurements();
            SmartDashboard.putBoolean("pigeonReady", false);
        }
        if (zeroingArm && armPigeon.PigeonReady())
        {
            zeroingArm = false;
            arm.setPosition(armPigeon.GetAngle());
            SmartDashboard.putBoolean("pigeonReady", true);
        }

        arm.checkEncoder();
        wrist.checkEncoder();
        elevator.checkEncoder();

        if (setPoint == ManipulatorSetPoint.cargo_pickup_back || setPoint == ManipulatorSetPoint.cargo_pickup_front)
        {
            robotCommand.SetSpearsClosed(false);
        }

        boolean scoreHatch = false;
        boolean grabHatch = false;

        boolean limitSwitchScore = limitSwitchesFired();

        scoreHatch = robotCommand.ManipulatorScore();
        if (!scoreHatch && robotCommand.LimitSwitchScore())
            scoreHatch = (limitSwitchScore || hatchPlacer.IsActive());

        grabHatch = robotCommand.HatchPickup();
        if (!grabHatch && robotCommand.LimitSwitchPickup())
            grabHatch = (limitSwitchScore || hatchGrabber.IsActive());

        if (grabHatch && scoreHatch)
        {
            scoreHatch = false;
        }

        if (!grabHatch && hatchCenter)
        {
            hatchCenterTimer++;
            if (hatchCenterTimer < 80)
                intake.runIntake(intake.Inputspeed);
            else
            {
                hatchCenter = false;
                hatchCenterTimer = 0;
                intake.Update(robotCommand);
            }
        }
        else
        {
            intake.Update(robotCommand);
        }

        // SmartDashboard.putString("hatchPlacer State", hatchPlacer.getState().name());
        // SmartDashboard.putString("hatchGrabber State",
        // hatchGrabber.getState().name());

        // SmartDashboard.putBoolean("AAA Manual Wrist", robotCommand.UseManualWrist());
        // SmartDashboard.putNumber("AAA Manual Wrist Value",
        // robotCommand.ManualWrist());

        if (robotCommand.UseManualWrist())
        {
            double wristOutput = robotCommand.ManualWrist();
            if (Math.abs(robotCommand.ManualWrist()) > .2)
            {
                wristOutput = Math.signum(robotCommand.ManualWrist()) * .2;
            }
            wrist.manual(wristOutput);
            manualWrist = true;
        }
        if (robotCommand.ZeroWrist())
        {
            wrist.disable();
            wrist.setPosition(arm.getPosition() - 134);
            manualWrist = true;
        }
        else if (!robotCommand.UseManualWrist())
        {
            manualWrist = false;
        }

        if (setPoint != null)
        {
            if (setPoint == ManipulatorSetPoint.hatch_pickup_front && !RobotState.getInstance().isSpearsClosed()
                    && !grabHatch)
            {
                setPoint = ManipulatorSetPoint.hatch_low_front;
            }
            IManipulatorSetPoint output = setPoint;

            if (scoreHatch)
            {
                output = hatchPlacer.GetPlacingSetPoint(setPoint);

                if (hatchPlacer.onTarget() && hatchPlacer.getState() == HatchPlacerState.Placing
                        || hatchPlacer.getState() == HatchPlacerState.RetractingArm
                        || hatchPlacer.getState() == HatchPlacerState.RetractingAll)
                {
                    robotCommand.SetSpearsClosed(true);
                    robotCommand.Rumble();
                }
            }
            else if (grabHatch)
            {
                output = hatchGrabber.GetPickupSetPoint(setPoint);
                hatchCenterTimer = 0;
                hatchCenter = true;
                if (hatchGrabber.IsActive() && hatchGrabber.getState() != HatchGrabberState.Pushing)
                {
                    robotCommand.SetSpearsClosed(false);
                }

                if (hatchGrabber.getState() == HatchGrabberState.Driving
                        || hatchGrabber.getState() == HatchGrabberState.Complete)
                {
                    robotCommand.Rumble();
                }
            }

            if (!grabHatch)
                hatchGrabber.Reset();
            if (!scoreHatch)
                hatchPlacer.Reset();

            output = CreateBumpedFlipperSetPoint(output, robotCommand.FrontFlipperBumpCount(),
                    robotCommand.BackFlipperBumpCount());

            SmartDashboard.putNumber("DrivePitch", robotState.getDrivePitch());

            if (Math.abs(robotState.getDrivePitch()) > 5
                    && robotCommand.ManipulatorSetPoint() == ManipulatorSetPoint.cargo_rocketHigh_back
                    || robotCommand.ManipulatorSetPoint() == ManipulatorSetPoint.cargo_rocketHigh_front)
            {
                output = new ManualManipulatorSetPoint(output.armAngle() - robotState.getDrivePitch(),
                        output.wristAngle(), output.elevatorHeight(), output.frontFlipper(), output.backFlipper());
            }

            if (robotCommand.FlipperCarry()
                    && (robotCommand.ManipulatorSetPoint() == ManipulatorSetPoint.hatch_high_back
                            || robotCommand.ManipulatorSetPoint() == ManipulatorSetPoint.hatch_mid_back
                            || robotCommand.ManipulatorSetPoint() == ManipulatorSetPoint.hatch_high_front
                            || robotCommand.ManipulatorSetPoint() == ManipulatorSetPoint.hatch_mid_front))
            {
                output = new ManualManipulatorSetPoint(output.armAngle(), output.wristAngle(), output.elevatorHeight(),
                        FlipperConstants.CARRY_FRONT, FlipperConstants.CARRY_BACK);
            }

            Control(output);
            robotActionsState.setDesiredManipulatorSetPoint(setPoint);
            robotActionsState.setManipulatorMoving(!onTarget(setPoint));
        }
        else
        {
            elevator.disable();
            arm.disable();
            frontFlipper.disable();
            backFlipper.disable();
            hatchPlacer.Reset();
            hatchGrabber.Reset();

            robotActionsState.setCommandedManipulatorSetPoint(null);
            robotActionsState.setDesiredManipulatorSetPoint(null);

            if (!manualWrist)
                wrist.disable();
        }

        intakePneumatics.Update(robotCommand);

        // SmartDashboard.putNumber("frontFlipper", frontFlipper.getPosition());
        // SmartDashboard.putNumber("backFlipper", backFlipper.getPosition());

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

    private boolean limitSwitchesFired()
    {
        DigitalInput leftLimit = (getArmSide(arm.getPosition()) == RobotSide.FRONT) ? frontLeftLimit : backLeftLimit;
        DigitalInput rightLimit = (getArmSide(arm.getPosition()) == RobotSide.FRONT) ? frontRightLimit : backRightLimit;

        robotState.setLeftLimitSwitch(leftLimit.get());
        robotState.setRightLimitSwitch(rightLimit.get());

        HotLogger.Log("leftLimit", leftLimit.get());
        HotLogger.Log("rightLimit", rightLimit.get());

        return leftLimit.get() && rightLimit.get();
    }

    private IManipulatorSetPoint CreateBumpedFlipperSetPoint(IManipulatorSetPoint setPoint, int frontCount,
            int backCount)
    {
        if (setPoint != null)
        {
            if (setPoint.frontFlipper() == FlipperConstants.HATCH_FRONT)
                setPoint = new ManualManipulatorSetPoint(setPoint.armAngle(), setPoint.wristAngle(),
                        setPoint.elevatorHeight(), setPoint.frontFlipper() + (frontCount * 3), setPoint.backFlipper());
            else if (setPoint.backFlipper() == FlipperConstants.HATCH_BACK)
                setPoint = new ManualManipulatorSetPoint(setPoint.armAngle(), setPoint.wristAngle(),
                        setPoint.elevatorHeight(), setPoint.frontFlipper(), setPoint.backFlipper() + (backCount * 3));
        }
        return setPoint;
    }

    public void UpdateRobotState()
    {
        robotState.setArmPosition(arm.getPosition());
        robotState.setElevatorPosition(elevator.getPosition());
        robotState.setWristPosition(wrist.getPosition());
        robotState.setFrontFlipperPosition(frontFlipper.getPosition());
        robotState.setBackFlipperPosition(backFlipper.getPosition());
        robotState.setSpearsClosed(intakePneumatics.SpearsClosed());

        robotActionsState.setHatchGrabberState(hatchGrabber.getState());
        robotActionsState.setHatchPlacerState(hatchPlacer.getState());
    }
}