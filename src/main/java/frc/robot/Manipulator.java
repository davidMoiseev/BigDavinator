/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.hotteam67.HotController;
import org.hotteam67.HotLogger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.WiringIDs;
import frc.robot.constants.ManipulatorSetPoint;

/**
 * Add your docs here.
 */
public class Manipulator {

    private enum InitailizationState {
        UNINITALIZED, CALIBRATING, READY
    }

    private enum ManipulatorState {
        intializing, packagePosition, outOfPackagePosition, transition, atTarget;
    }

    private enum RobotSide {
        FRONT, BACK
    }

    private Elevator elevator;
    private Intake intake;
    private IntakePneumatics pneumaticIntake;
    private Wrist wrist;
    private Arm arm;

    private HotController driver;
    private HotController operator;
    private ArmPigeon armPigeon;
    private DriveTrain drivetrain;
    private InitailizationState initailizationState;
    private ManipulatorState manipulatorState;
    private boolean startButtonPrevious = true;
    private boolean backButtonPrevious = true;
    private boolean commandToBack = false;
    private boolean isLeftTriggerPressed;
    private boolean isRightTriggerPressed;
    private boolean inCarry = false;

    private double prevElevHeight;
    private double prevArmAngle;
    private double prevWristAngle;
    public double armAngleBalance;

    private final double ELEVATOR_CLEAR_HEIGHT = 18;
    private final double ARM_TOLERANCE = 3; // need
    private final double WRIST_TOLERANCE = 3; // need
    private final double ARM_LENGTH = 21;
    private final double WRIST_LENGTH = 14;
    private final double ELEVATOR_LENGTH = 19;
    private final double FRAME_COLLISION = 5;
    private final double SUPPORTS_COLLISION = 14; // need?
    private final double FRAME_LENGTH = 16; // 14
    private final double intakeHeight = 13.5;

    public Manipulator(HotController operator, HotController driver, TalonSRX rightElevator, TalonSRX intake,
            DriveTrain drivetrain) {
        this.elevator = new Elevator(new TalonSRX(WiringIDs.LEFT_ELEVATOR), rightElevator);
        this.wrist = new Wrist(WiringIDs.WRIST);
        this.arm = new Arm(WiringIDs.SHOULDER);
        this.armPigeon = new ArmPigeon(WiringIDs.PIGEON_ARM);
        this.intake = new Intake(driver);
        this.pneumaticIntake = new IntakePneumatics(driver);

        this.operator = operator;
        this.driver = driver;
        this.drivetrain = drivetrain;

        RestartInitialization();
        manipulatorState = ManipulatorState.intializing;

    }

    public void InitializeTalons() {
        elevator.initialize();
        wrist.initialize();
        arm.initialize();
    }

    public void RestartInitialization() {
        initailizationState = InitailizationState.UNINITALIZED;
    }

    public boolean isReady() {
        return initailizationState == InitailizationState.READY;
    }

    public void RunManipulatorInitialization() {

        if (initailizationState == InitailizationState.UNINITALIZED) {
            armPigeon.RestartCalibration();
            initailizationState = InitailizationState.CALIBRATING;
        }
        if (initailizationState == InitailizationState.CALIBRATING) {

            armPigeon.CalibratePigeon();
            if (armPigeon.PigeonReady()) {
                initailizationState = InitailizationState.READY;
                manipulatorState = ManipulatorState.packagePosition;
                arm.setPosition(armPigeon.GetAngle());
                wrist.setPosition(armPigeon.GetAngle() - 134);
                elevator.zeroSensors();

            }
        }

    }

    public void DisplaySensors() {
        elevator.displaySensorsValue();
        arm.displaySensorsValue();
        wrist.displaySensorsValue();
    }

    private void Control(ManipulatorSetPoint targetPosition) {
        // If the target position will collide with the frame or support beams
        if (manipulatorState == ManipulatorState.intializing) {
            elevator.disable();
            arm.disable();
            wrist.disable();
        } else if (manipulatorState == ManipulatorState.packagePosition) {
            elevator.disable();
            arm.disable();
            wrist.disable();
            if (targetPosition != null) {
                manipulatorState = ManipulatorState.outOfPackagePosition;
            }
        } else if (manipulatorState == ManipulatorState.outOfPackagePosition) {
            double tmpArm = arm.getPosition();
            elevator.setTarget(ManipulatorSetPoint.firstPosition);
            if (elevator.reachedTarget()) {
                arm.setTarget(tmpArm);
                wrist.setTarget(ManipulatorSetPoint.firstPosition);
                if (wrist.reachedTarget()) {
                    arm.setTarget(ManipulatorSetPoint.firstPosition);
                    if (arm.reachedTarget()) {
                        manipulatorState = ManipulatorState.atTarget;
                        // manipulatorState = ManipulatorState.transition;
                    }
                }
            }
        }

        if (manipulatorState == ManipulatorState.transition) {
            setTargets(targetPosition, targetPosition, targetPosition);
        }

        if (manipulatorState == ManipulatorState.atTarget) {
            if (willCollideWithFrame(targetPosition.elevatorHeight(), targetPosition.armAngle(),
                    targetPosition.wristAngle())
                    || willCollideWithSupports(targetPosition.elevatorHeight(), targetPosition.armAngle(),
                            targetPosition.wristAngle())) {

                setTargets(ManipulatorSetPoint.limit_front_high, ManipulatorSetPoint.limit_front_high,
                        ManipulatorSetPoint.limit_front_high);
                HotLogger.Log("AA debug Arm", 0);
                System.out.println("LogStuff: 0");
            }
            // If the elevator is currently over the clear height, set any position (as long
            // as it doesn't collide with itself)
            else if (elevator.getPosition() > ELEVATOR_CLEAR_HEIGHT) {
                setTargets(targetPosition, targetPosition, targetPosition);
                HotLogger.Log("AA debug Arm", 1);
                System.out.println("LogStuff: 1");
            }
            // If it isn't
            else {
                // Same side
                if (getArmSide(targetPosition.armAngle()) == getArmSide(arm.getPosition())) {
                    // Front side
                    if (getArmSide(targetPosition.armAngle()) == RobotSide.FRONT) {
                        // If arm is in the range where it won't collide with the frame or elevator
                        // supports
                        if (targetPosition.armAngle() > ManipulatorSetPoint.limit_front_high.armAngle()
                                && targetPosition.armAngle() < ManipulatorSetPoint.limit_front_low.armAngle()) {
                            setTargets(targetPosition, targetPosition, targetPosition);
                            HotLogger.Log("AA debug Arm", 3);
                            System.out.println("LogStuff: 3");
                        }
                        // compare current locations, if the target is out of bounds, set to a high
                        // value.
                        // if the current is about to collide, then for the supports, adjust the arm, if
                        // frame, the elev height
                        else {
                            if (willCollideWithFrame(elevator.getPosition(), arm.getPosition(), wrist.getPosition())
                                    || (willCollideWithSupports(elevator.getPosition(), arm.getPosition(),
                                            wrist.getPosition()))) {
                                setTargets(targetPosition, targetPosition, targetPosition);
                                // setTargets(prevElevHeight, prevArmAngle, targetPosition.wristAngle());
                                HotLogger.Log("AA debug Arm", 4);
                                System.out.println("LogStuff: 4");
                            } else {
                                setTargets(targetPosition, targetPosition, targetPosition);
                                HotLogger.Log("AA debug Arm", 5);
                                System.out.println("LogStuff: 5");
                            }
                        }
                    }
                    // Back side
                    else {
                        // If arm is in the range where it won't collide with the frame or elevator
                        // supports
                        if (targetPosition.armAngle() > ManipulatorSetPoint.limit_back_high.armAngle()
                                && targetPosition.armAngle() < ManipulatorSetPoint.limit_back_low.armAngle()) {
                            setTargets(targetPosition, targetPosition, targetPosition);
                            HotLogger.Log("AA debug Arm", 6);
                            System.out.println("LogStuff: 6");
                        }
                        // compare current locations, if the target is out of bounds, set to a high
                        // value.
                        // if the current is about to collide, then for the supports, adjust the arm, if
                        // frame, the elev height
                        else {
                            if (willCollideWithFrame(elevator.getPosition(), arm.getPosition(), wrist.getPosition())
                                    || (willCollideWithSupports(elevator.getPosition(), arm.getPosition(),
                                            wrist.getPosition()))) {
                                setTargets(prevElevHeight, prevArmAngle, targetPosition.wristAngle());
                                HotLogger.Log("AA debug Arm", 7);
                                System.out.println("LogStuff: 7");
                            } else {
                                setTargets(targetPosition, targetPosition, targetPosition);
                                HotLogger.Log("AA debug Arm", 8);
                                System.out.println("LogStuff: 8");
                            }
                        }
                    }
                }
                // Switching sides
                else {
                    // Swapping from front to back
                    if (getArmSide(targetPosition.armAngle()) == RobotSide.FRONT) {
                        // If the target will clear the top
                        if (targetPosition.elevatorHeight() > ELEVATOR_CLEAR_HEIGHT) {
                            // command everything to their positions, except for the arm, until the elevator
                            // clears the top
                            if ((arm.getPosition() + ARM_TOLERANCE) > ManipulatorSetPoint.limit_front_high.armAngle()) {
                                setTargets(targetPosition, ManipulatorSetPoint.limit_front_high, targetPosition);
                                HotLogger.Log("AA debug Arm", 9);
                                System.out.println("LogStuff: 9");
                            } else {
                                // Making sure that the arm is safe before we move the wrist
                                setTargets(prevElevHeight, ManipulatorSetPoint.limit_front_high.armAngle(),
                                        prevWristAngle);
                                HotLogger.Log("AA debug Arm", 10);
                                System.out.println("LogStuff: 10");
                            }
                        }
                        // If the target won't clear the top
                        else {
                            // arm and wrist are in position to move to the other side
                            if (isArmSafe() && isWristSafe()) {
                                setTargets(targetPosition, ManipulatorSetPoint.limit_back_high,
                                        ManipulatorSetPoint.limit_back_high);
                                HotLogger.Log("AA debug Arm", 11);
                                System.out.println("LogStuff: 11");
                            }
                            // If the arm is outside the "safe zone"
                            else if ((arm.getPosition() + ARM_TOLERANCE) > ManipulatorSetPoint.limit_front_high
                                    .armAngle()) {
                                // if the elevator is going up, move it in addition to straightening the wrist
                                // and arm
                                if (targetPosition.elevatorHeight() > ManipulatorSetPoint.limit_front_high
                                        .elevatorHeight()) {
                                    setTargets(targetPosition, ManipulatorSetPoint.limit_front_high,
                                            ManipulatorSetPoint.limit_back_high);
                                    HotLogger.Log("AA debug Arm", 12);
                                    System.out.println("LogStuff: 12");
                                }
                                // if the elevator is going down, make sure that the arm and wrist don't ram the
                                // robot
                                else {
                                    setTargets(prevElevHeight, ManipulatorSetPoint.limit_front_high.armAngle(),
                                            ManipulatorSetPoint.limit_back_high.wristAngle());
                                    HotLogger.Log("AA debug Arm", 13);
                                    System.out.println("LogStuff: 13");
                                }
                            } else {
                                // Making sure the arm is safe before we move the wrist
                                setTargets(prevElevHeight, ManipulatorSetPoint.limit_front_high.armAngle(),
                                        prevWristAngle);
                                HotLogger.Log("AA debug Arm", 14);
                                System.out.println("LogStuff: 14");
                            }
                        }
                    }
                    // Swapping from back to front
                    else {
                        // If the target will clear the top
                        if (targetPosition.elevatorHeight() > ELEVATOR_CLEAR_HEIGHT) {
                            // command everything to their positions, except for the arm, until the elevator
                            // clears the top
                            if ((arm.getPosition() - ARM_TOLERANCE) > ManipulatorSetPoint.limit_back_high.armAngle()) {
                                setTargets(targetPosition, ManipulatorSetPoint.limit_back_high, targetPosition);
                                HotLogger.Log("AA debug Arm", 15);
                                System.out.println("LogStuff: 15");
                            } else {
                                // Making sure that the arm is safe before we move the wrist
                                setTargets(prevElevHeight, ManipulatorSetPoint.limit_back_high.armAngle(),
                                        prevWristAngle);
                                HotLogger.Log("AA debug Arm", 16);
                                System.out.println("LogStuff: 16");
                            }
                        } else {
                            // arm and wrist are in position to move to the other side
                            if (isArmSafe() && isWristSafe()) {
                                setTargets(targetPosition, ManipulatorSetPoint.limit_front_high,
                                        ManipulatorSetPoint.limit_front_high);
                                HotLogger.Log("AA debug Arm", 17);
                                System.out.println("LogStuff: 17");
                            }
                            // If the arm is outside the "safe zone"
                            else if ((arm.getPosition() - ARM_TOLERANCE) > ManipulatorSetPoint.limit_back_high
                                    .armAngle()) {
                                // if the elevator is going up, move it in addition to straightening the wrist
                                // and arm
                                if (targetPosition.elevatorHeight() > ManipulatorSetPoint.limit_back_high
                                        .elevatorHeight()) {
                                    setTargets(targetPosition, ManipulatorSetPoint.limit_back_high,
                                            ManipulatorSetPoint.limit_front_high);
                                    HotLogger.Log("AA debug Arm", 18);
                                    System.out.println("LogStuff: 18");
                                } else {
                                    // if the elevator is going down, make sure that the arm and wrist don't ram the
                                    // robot
                                    setTargets(prevElevHeight, ManipulatorSetPoint.limit_back_high.armAngle(),
                                            ManipulatorSetPoint.limit_front_high.wristAngle());
                                    HotLogger.Log("AA debug Arm", 19);
                                    System.out.println("LogStuff: 19");
                                }
                            } else {
                                // Making sure that the arm is safe before we move the wrist
                                setTargets(prevElevHeight, ManipulatorSetPoint.limit_back_high.armAngle(),
                                        prevWristAngle);
                                HotLogger.Log("AA debug Arm", 20);
                                System.out.println("LogStuff: 20");
                            }
                        }
                    }
                }
            }
        }
        prevArmAngle = arm.getPosition();
        prevWristAngle = wrist.getPosition();
    }

    private void setTargets(double elevTarget, double armTarget, double wristTarget) {
        elevator.setTarget(elevTarget);
        arm.setTarget(armTarget);
        wrist.setTarget(wristTarget);
    }

    private void setTargets(ManipulatorSetPoint elevTarget, ManipulatorSetPoint armTarget,
            ManipulatorSetPoint wristTarget) {
        elevator.setTarget(elevTarget);
        arm.setTarget(armTarget);
        wrist.setTarget(wristTarget);
    }

    // This is the side the arm is on
    private RobotSide getArmSide(double armAngle) {
        if (armAngle > -5.0)
            return RobotSide.FRONT;
        return RobotSide.BACK;
    }

    private boolean isArmSafe() {
        // May have to tweak a tolerance
        return (arm.getPosition() > ManipulatorSetPoint.limit_back_high.armAngle()
                && arm.getPosition() < ManipulatorSetPoint.limit_front_high.armAngle());
    }

    private boolean isWristSafe() {
        // May have to tweak a tolerance
        return (wrist.getPosition() > ManipulatorSetPoint.limit_back_high.wristAngle()
                && wrist.getPosition() < ManipulatorSetPoint.limit_front_high.wristAngle());
    }

    private boolean willCollideWithFrame(double elevHeight, double armAngle, double wristAngle) {
        if ((arm.getPosition() > 90.0 && widthManipulator(armAngle, wristAngle) < FRAME_LENGTH)
                || (arm.getPosition() < -90.0 && widthManipulator(armAngle, wristAngle) > -FRAME_LENGTH))
            return ((heightManipulator(elevHeight, arm.getPosition(), wrist.getPosition())
                    + 13.5 / 2 * Math.sin(Math.toRadians(wrist.getPosition())) < FRAME_COLLISION)
                    || (heightManipulator(elevHeight, arm.getPosition(), wrist.getPosition())
                            - 13.5 / 2 * Math.sin(Math.toRadians(wrist.getPosition())) < FRAME_COLLISION));
        return false;
    }

    private boolean willCollideWithSupports(double elevHeight, double armAngle, double wristAngle) {

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

        if ((yBottom < 44 && Math.abs(xBottom) < 5) || (yTop < 44 && Math.abs(xTop) < 5)) {
            return true;
        } else {
            return false;
        }
    }

    private double lengthWristY(double angle) {
        // in case we need to adjust this
        return WRIST_LENGTH * Math.cos(Math.toRadians(angle));
    }

    private double lengthWristX(double angle) {
        // in case we need to adjust this again
        return WRIST_LENGTH * Math.sin(Math.toRadians(angle));
    }

    private double heightManipulator(double elvatorHeight, double armAngle, double wristAngle) {
        return elvatorHeight + ELEVATOR_LENGTH + lengthWristY(wristAngle)
                + ARM_LENGTH * Math.cos(Math.toRadians(armAngle));
    }

    private double widthManipulator(double armAngle, double wristAngle) {
        return ARM_LENGTH * Math.sin(Math.toRadians(armAngle)) + lengthWristX(wristAngle);
    }

    public void Update() {
        intake.Update();
        pneumaticIntake.Update();
        ManipulatorSetPoint frontTargetPosition = null;
        ManipulatorSetPoint backTargetPosition = null;

        if (operator.getLeftTrigger() >= .25) {
            isLeftTriggerPressed = true;
        } else {
            isLeftTriggerPressed = false;
        }

        if (operator.getRightTrigger() >= .25) {
            isRightTriggerPressed = true;
        } else {
            isRightTriggerPressed = false;
        }

        if (operator.getButtonBack()) {
            // flip code
        } else if (operator.getButtonStart()) {
            // flip code
        } else if (operator.getButtonX()) {
            frontTargetPosition = ManipulatorSetPoint.carry_front;
            backTargetPosition = ManipulatorSetPoint.carry_back;

            if (commandToBack == false) {
                SmartDashboard.putBoolean("Command To Front", commandToBack);
                if (arm.reachedTarget() == true) {
                    arm.isInCarry(true, ManipulatorSetPoint.carry_front);
                    SmartDashboard.putBoolean("isInCarry Front", inCarry);
                } else {
                    arm.isInCarry(false, ManipulatorSetPoint.carry_front);
                    SmartDashboard.putBoolean("Going To Carry Front", inCarry);
                }
            } else if (commandToBack == true) {
                SmartDashboard.putBoolean("Command To Back", commandToBack);
                if (arm.reachedTarget() == true) {
                    arm.isInCarry(true, ManipulatorSetPoint.carry_back);
                    SmartDashboard.putBoolean("isInCarry Back", inCarry);
                } else {
                    arm.isInCarry(false, ManipulatorSetPoint.carry_back);
                    SmartDashboard.putBoolean("Going To Carry Back", inCarry);
                }
            }
        } else if (operator.getButtonA()) {
            frontTargetPosition = ManipulatorSetPoint.hatch_low_front;
            backTargetPosition = ManipulatorSetPoint.hatch_low_back;
        } else if (operator.getButtonB()) {
            frontTargetPosition = ManipulatorSetPoint.hatch_mid_front;
            backTargetPosition = ManipulatorSetPoint.hatch_mid_back;
        }
        /*
         * else if (operator.getButtonY()) { frontTargetPosition =
         * ManipulatorSetPoint.hatch_high_front; backTargetPosition =
         * ManipulatorSetPoint.hatch_high_back; }
         */ else if (operator.getButtonLeftBumper()) {
            frontTargetPosition = ManipulatorSetPoint.cargo_rocketLow_front;
            backTargetPosition = ManipulatorSetPoint.cargo_rocketLow_back;
        } else if (isLeftTriggerPressed == true) {
            frontTargetPosition = ManipulatorSetPoint.cargo_rocketMid_front;
            backTargetPosition = ManipulatorSetPoint.cargo_rocketMid_back;
        }
        /*
         * else if (isRightTriggerPressed == true) { frontTargetPosition =
         * ManipulatorSetPoint.cargo_rocketHigh_front; backTargetPosition =
         * ManipulatorSetPoint.cargo_rocketHigh_back; }
         */ else if (operator.getButtonRightBumper()) {
            frontTargetPosition = ManipulatorSetPoint.cargo_shuttle_front;
            backTargetPosition = ManipulatorSetPoint.cargo_shuttle_back;
        } else if (operator.getButtonLeftStick()) {
            frontTargetPosition = ManipulatorSetPoint.cargo_pickup_front;
            backTargetPosition = ManipulatorSetPoint.cargo_pickup_back;
        } else if (operator.getButtonRightStick()) {
            frontTargetPosition = ManipulatorSetPoint.cargo_pickup_front;
            backTargetPosition = ManipulatorSetPoint.cargo_pickup_back;
        }

        if (operator.getButtonStart() && !startButtonPrevious) {
            commandToBack = !commandToBack;
        }

        if (frontTargetPosition != null || backTargetPosition != null) {
            if (commandToBack) {
                Control(backTargetPosition);
            } else {
                Control(frontTargetPosition);
            }
            SmartDashboard.putBoolean("Disabled thing", false);
        } else {
            elevator.disable();
            arm.disable();
            wrist.disable();
            SmartDashboard.putBoolean("Disabled thing", true);
        }

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

        prevElevHeight = elevator.getPosition();
        prevArmAngle = arm.getPosition();
        prevWristAngle = wrist.getPosition();
    }
}