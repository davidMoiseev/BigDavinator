/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.hotteam67.HotController;

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
    private boolean commandToBack = false;

    private double prevElevHeight;
    private double prevArmAngle;
    private double prevWristAngle;

    private final double ELEVATOR_CLEAR_HEIGHT = 18;
    private final double ARM_TOLERANCE = 3; //need
    private final double WRIST_TOLERANCE = 3; //need
    private final double ARM_LENGTH = 21;
    private final double WRIST_LENGTH = 14;
    private final double ELEVATOR_LENGTH = 19; 
    private final double FRAME_COLLISION = 20;
    private final double SUPPORTS_COLLISION = 14; //need?
    private final double FRAME_LENGTH = 16; //14

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
        //If the target position will collide with the frame or support beams
        if (manipulatorState == ManipulatorState.intializing) {
            elevator.disable();
            arm.disable();
            wrist.disable();
        }
        else if (manipulatorState == ManipulatorState.packagePosition) {
            elevator.disable();
            arm.disable();
            wrist.disable();
            if (targetPosition != null) {
                manipulatorState = ManipulatorState.outOfPackagePosition;
            }
        }
        else if (manipulatorState == ManipulatorState.outOfPackagePosition)
        {
            elevator.setTarget(ManipulatorSetPoint.firstPostion);
            if (elevator.reachedTarget())
            {
                wrist.setTarget(ManipulatorSetPoint.firstPostion);
                if (wrist.reachedTarget())
                {
                    arm.setTarget(ManipulatorSetPoint.firstPostion);
                    if (arm.reachedTarget())
                    {
                        //manipulatorState = ManipulatorState.atTarget;
                    }
                }
            }
        }
        
        if (manipulatorState == ManipulatorState.atTarget || manipulatorState == ManipulatorState.transition)
        {
            if (willCollideWithFrame(targetPosition.elevatorHeight(), targetPosition.armAngle(),
                    targetPosition.wristAngle())
                    || willCollideWithSupports(targetPosition.elevatorHeight(), targetPosition.armAngle(),
                            targetPosition.wristAngle()))
            {

                setTargets(ManipulatorSetPoint.limit_front_high, ManipulatorSetPoint.limit_front_high,
                        ManipulatorSetPoint.limit_front_high);
            }
            //If the elevator is currently over the clear height, set any position (as long as it doesn't collide with itself)
            else if (elevator.getPosition() > ELEVATOR_CLEAR_HEIGHT)
            {
                setTargets(targetPosition, targetPosition, targetPosition);
            }
            //If it isn't
            else
            {
                //Same side
                if (getArmSide(targetPosition.armAngle()) == getArmSide(arm.getPosition()))
                {
                    //Front side
                    if (getArmSide(targetPosition.armAngle()) == RobotSide.FRONT)
                    {
                        //If arm is in the range where it won't collide with the frame or elevator supports
                        if (targetPosition.armAngle() > ManipulatorSetPoint.limit_front_high.armAngle()
                                && targetPosition.armAngle() < ManipulatorSetPoint.limit_front_low.armAngle())
                        {
                            setTargets(targetPosition, targetPosition, targetPosition);
                        }
                        //compare current locations, if the target is out of bounds, set to a high value.
                        //if the current is about to collide, then for the supports, adjust the arm, if frame, the elev height
                        else
                        {
                            if (willCollideWithFrame(elevator.getPosition(), arm.getPosition(), wrist.getPosition())
                                    || (willCollideWithSupports(elevator.getPosition(), arm.getPosition(),
                                            wrist.getPosition())))
                            {
                                setTargets(prevElevHeight, prevArmAngle, targetPosition.wristAngle());
                            }
                            else
                            {
                                setTargets(targetPosition, targetPosition, targetPosition);
                            }
                        }
                    }
                    //Back side
                    else
                    {
                        //If arm is in the range where it won't collide with the frame or elevator supports
                        if (targetPosition.armAngle() > ManipulatorSetPoint.limit_back_high.armAngle()
                                && targetPosition.armAngle() < ManipulatorSetPoint.limit_back_low.armAngle())
                        {
                            setTargets(targetPosition, targetPosition, targetPosition);
                        }
                        //compare current locations, if the target is out of bounds, set to a high value.
                        //if the current is about to collide, then for the supports, adjust the arm, if frame, the elev height
                        else
                        {
                            if (willCollideWithFrame(elevator.getPosition(), arm.getPosition(), wrist.getPosition())
                                    || (willCollideWithSupports(elevator.getPosition(), arm.getPosition(),
                                            wrist.getPosition())))
                            {
                                setTargets(prevElevHeight, prevArmAngle, targetPosition.wristAngle());
                            }
                            else
                            {
                                setTargets(targetPosition, targetPosition, targetPosition);
                            }
                        }
                    }
                }
                //Switching sides
                else
                {
                    //Swapping from front to back
                    if (getArmSide(targetPosition.armAngle()) == RobotSide.FRONT)
                    {
                        //If the target will clear the top
                        if (targetPosition.elevatorHeight() > ELEVATOR_CLEAR_HEIGHT)
                        {
                            //command everything to their positions, except for the arm, until the elevator clears the top
                            if ((arm.getPosition() + ARM_TOLERANCE) > ManipulatorSetPoint.limit_front_high.armAngle())
                            {
                                setTargets(targetPosition, ManipulatorSetPoint.limit_front_high, targetPosition);
                            }
                            else
                            {
                                //Making sure that the arm is safe before we move the wrist
                                setTargets(prevElevHeight, ManipulatorSetPoint.limit_front_high.armAngle(),
                                        prevWristAngle);
                            }
                        }
                        //If the target won't clear the top
                        else
                        {
                            //arm and wrist are in position to move to the other side
                            if (isArmSafe() && isWristSafe())
                            {
                                setTargets(targetPosition, ManipulatorSetPoint.limit_back_high,
                                        ManipulatorSetPoint.limit_back_high);
                            }
                            //If the arm is outside the "safe zone"
                            else if ((arm.getPosition() + ARM_TOLERANCE) > ManipulatorSetPoint.limit_front_high
                                    .armAngle())
                            {
                                //if the elevator is going up, move it in addition to straightening the wrist and arm
                                if (targetPosition.elevatorHeight() > ManipulatorSetPoint.limit_front_high
                                        .elevatorHeight())
                                {
                                    setTargets(targetPosition, ManipulatorSetPoint.limit_front_high,
                                            ManipulatorSetPoint.limit_back_high);
                                }
                                //if the elevator is going down, make sure that the arm and wrist don't ram the robot
                                else
                                {
                                    setTargets(prevElevHeight, ManipulatorSetPoint.limit_front_high.armAngle(),
                                            ManipulatorSetPoint.limit_back_high.wristAngle());
                                }
                            }
                            else
                            {
                                //Making sure the arm is safe before we move the wrist
                                setTargets(prevElevHeight, ManipulatorSetPoint.limit_front_high.armAngle(),
                                        prevWristAngle);
                            }
                        }
                    }
                    //Swapping from back to front
                    else
                    {
                        //If the target will clear the top
                        if (targetPosition.elevatorHeight() > ELEVATOR_CLEAR_HEIGHT)
                        {
                            //command everything to their positions, except for the arm, until the elevator clears the top
                            if ((arm.getPosition() - ARM_TOLERANCE) > ManipulatorSetPoint.limit_back_high.armAngle())
                            {
                                setTargets(targetPosition, ManipulatorSetPoint.limit_back_high, targetPosition);
                            }
                            else
                            {
                                //Making sure that the arm is safe before we move the wrist
                                setTargets(prevElevHeight, ManipulatorSetPoint.limit_back_high.armAngle(),
                                        prevWristAngle);
                            }
                        }
                        else
                        {
                            //arm and wrist are in position to move to the other side
                            if (isArmSafe() && isWristSafe())
                            {
                                setTargets(targetPosition, ManipulatorSetPoint.limit_front_high,
                                        ManipulatorSetPoint.limit_front_high);
                            }
                            //If the arm is outside the "safe zone"
                            else if ((arm.getPosition() - ARM_TOLERANCE) > ManipulatorSetPoint.limit_back_high
                                    .armAngle())
                            {
                                //if the elevator is going up, move it in addition to straightening the wrist and arm
                                if (targetPosition.elevatorHeight() > ManipulatorSetPoint.limit_back_high
                                        .elevatorHeight())
                                {
                                    setTargets(targetPosition, ManipulatorSetPoint.limit_back_high,
                                            ManipulatorSetPoint.limit_front_high);
                                }
                                else
                                {
                                    //if the elevator is going down, make sure that the arm and wrist don't ram the robot
                                    setTargets(prevElevHeight, ManipulatorSetPoint.limit_back_high.armAngle(),
                                            ManipulatorSetPoint.limit_front_high.wristAngle());
                                }
                            }
                            else
                            {
                                //Making sure that the arm is safe before we move the wrist
                                setTargets(prevElevHeight, ManipulatorSetPoint.limit_back_high.armAngle(),
                                        prevWristAngle);
                            }
                        }
                    }
                }
            }
        }
        SmartDashboard.putBoolean("COLLIDE FRAME", willCollideWithFrame(elevator.getPosition(),
            arm.getPosition(), wrist.getPosition()));
        SmartDashboard.putBoolean("COLLIDE SUPPORTS", willCollideWithSupports(elevator.getPosition(), arm.getPosition(),
                wrist.getPosition()));
        SmartDashboard.putNumber("Wrist Width", widthManipulator(arm.getPosition(), wrist.getPosition()));
        prevElevHeight = elevator.getPosition();
        prevArmAngle = arm.getPosition();
        prevWristAngle = wrist.getPosition();
    }

    private void setTargets(double elevTarget, double armTarget, double wristTarget) {
        elevator.setTarget(elevTarget);
        arm.setTarget(armTarget);
        wrist.setTarget(wristTarget);
    }

    private void setTargets(ManipulatorSetPoint elevTarget, ManipulatorSetPoint armTarget, ManipulatorSetPoint wristTarget) {
        elevator.setTarget(elevTarget);
        arm.setTarget(armTarget);
        wrist.setTarget(wristTarget);
    }

    //This is the side the arm is on
    private RobotSide getArmSide(double armAngle) {
        if (armAngle > 0.0)
            return RobotSide.FRONT;
        return RobotSide.BACK;
    }

    private boolean isArmSafe() {
        //May have to tweak a tolerance
        return (arm.getPosition() > ManipulatorSetPoint.limit_back_high.armAngle() && 
            arm.getPosition() < ManipulatorSetPoint.limit_front_high.armAngle());
    }

    private boolean isWristSafe() {
        //May have to tweak a tolerance
        return (wrist.getPosition() > ManipulatorSetPoint.limit_back_high.wristAngle() && 
            wrist.getPosition() < ManipulatorSetPoint.limit_front_high.wristAngle());
    }

    private boolean willCollideWithFrame(double elevHeight, double armAngle, double wristAngle) {
        if((arm.getPosition() > 90.0 || arm.getPosition() < -90.0) && 
                widthManipulator(armAngle, wristAngle) < FRAME_LENGTH)
            
            return ELEVATOR_LENGTH + elevHeight - lengthManipulator(armAngle, wristAngle) < FRAME_COLLISION;
        return false;
    }

    private boolean willCollideWithSupports(double elevHeight, double armAngle, double wristAngle) {
        if(armAngle > 120.0 || armAngle < -120.0 || (elevHeight < ELEVATOR_CLEAR_HEIGHT && 
            wristAngle > ManipulatorSetPoint.limit_front_high.wristAngle()) || (elevHeight < ELEVATOR_CLEAR_HEIGHT &&
            wristAngle < ManipulatorSetPoint.limit_back_high.wristAngle()))
            
            return widthManipulator(armAngle, wristAngle)  < SUPPORTS_COLLISION;
        return false;
    }

    private double lengthWristY(double angle) {
        //in case we need to adjust this
        return WRIST_LENGTH * Math.abs(Math.cos(Math.toRadians(angle)));
    }

    private double lengthWristX(double angle) {
        //in case we need to adjust this again
        return WRIST_LENGTH * Math.abs(Math.sin(Math.toRadians(angle)));
    }

    private double lengthManipulator(double armAngle, double wristAngle) {
        return (ARM_LENGTH + lengthWristY(wristAngle)) * Math.abs(Math.cos(Math.toRadians(armAngle)));
    }

    private double widthManipulator(double armAngle, double wristAngle) {
        return ARM_LENGTH * Math.abs(Math.sin(Math.toRadians(armAngle))) - lengthWristX(wristAngle);
    }

    public void Update() {
        intake.Update();
        pneumaticIntake.Update();
        ManipulatorSetPoint frontTargetPosition = null;
        ManipulatorSetPoint backTargetPosition = null;

        if (operator.getButtonBack()) {
            frontTargetPosition = ManipulatorSetPoint.carry_front;
            backTargetPosition = ManipulatorSetPoint.carry_back;
        } else if (operator.getButtonB()) {
            frontTargetPosition = ManipulatorSetPoint.hatch_mid_front;
            backTargetPosition = ManipulatorSetPoint.hatch_mid_back;
        } else if (operator.getButtonA()) {
            frontTargetPosition = ManipulatorSetPoint.cargo_pickup_front;
            backTargetPosition = ManipulatorSetPoint.cargo_pickup_back;
        } else if (operator.getButtonX()) {
            frontTargetPosition = ManipulatorSetPoint.cargo_rocketMid_front;
            backTargetPosition = ManipulatorSetPoint.cargo_rocketMid_back;
        } else if (operator.getButtonY()) {
            frontTargetPosition = ManipulatorSetPoint.cargo_rocketLow_front;
            backTargetPosition = ManipulatorSetPoint.cargo_rocketLow_back;
        } else if (operator.getButtonLeftBumper()) {
            frontTargetPosition = ManipulatorSetPoint.hatch_low_front;
            backTargetPosition = ManipulatorSetPoint.hatch_low_back;
        } else if (operator.getButtonRightBumper()) {

        } else if (operator.getButtonLeftStick()) {

        } else if (operator.getButtonRightStick()) {

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
        } else {
            elevator.disable();
            arm.disable();
            wrist.disable();
        }

        startButtonPrevious = operator.getButtonStart();

        prevElevHeight = elevator.getPosition();
        prevArmAngle = arm.getPosition();
        prevWristAngle = wrist.getPosition();
    }
}