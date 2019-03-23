package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.hotteam67.HotLogger;
import org.hotteam67.HotPathFollower;
import org.hotteam67.HotPathFollower.State;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.TeleopCommandProvider;
import frc.robot.constants.WiringIDs;

public class DriveTrain implements IPigeonWrapper
{
    public static final double WHEEL_DIAMETER = 4.0 * 2.54 / 100;
    public static final int TICKS_PER_REVOLUTION = 1;

    public static final double ENCODER_TO_REVS = (50.0 / 12.0) * (42.0 / 24.0);

    public static final double SECOND_ENCODER_TO_REVS = 4096 * (42.0 / 24.0);

    // Recorded max velocity: 3000 units per 100 ms
    // 21,080.986
    public static final double TICKS_PER_METER = 22000;

    public static final double MAX_VELOCITY_TICKS = 8000;
    // Max velocity in m/s
    public static final double MAX_VELOCITY = (MAX_VELOCITY_TICKS / TICKS_PER_METER) * 10;

    /**
     * Primary motor controllers
     */
    private final CANSparkMax rightMotor;
    private final CANSparkMax leftMotor;

    /**
     * Following motor controllers
     */
    private final CANSparkMax rightFollower;
    private final CANSparkMax leftFollower;

    private final CANSparkMax hDriveMotor;

    private VictorSPX leftClimbMotor;
    private VictorSPX rightClimbMotor;
    private Solenoid climber;

    private boolean allowClimb = false;
    private boolean climbDeployed = false;

    private final PigeonIMU pigeon;
    VisionMotion vmotion = new VisionMotion();
    private final TalonSRX rightEncoder;
    private final TalonSRX leftEncoder;
    private double[] xyz_dps = new double[3];
    private double hDrivePrevious;
    private HDriveState hDriveState = HDriveState.Off;

    private enum HDriveState
    {
        Off, Spike, Ramping
    }

    // values without offset
    private double leftEncoderValue = 0;
    private double rightEncoderValue = 0;

    private final HotPathFollower pathFollower;
    public boolean isHDown;
    public double previousCanSeeTarget;
    public double prevprevCanSeeTarget;
    public static final double pGain = 0.04;
    public static final double iGain = 0.000008;
    public static final double dGain = 0.0;
    public double integral = 0;
    public double speed = 0;
    public double Lspeed;
    public double Rspeed;
    public double currentYaw;
    public double singleRotationYaw;
    public int state;

    /**
     * Motion Profiling Constants
     */
    public static final class POS_PIDVA
    {
        public static final double P = .75;
        public static final double I = 0;
        public static final double D = 0;
        public static final double V = 1.0 / MAX_VELOCITY; // Velocity feed forward
        public static final double A = 0; // Acceleration gain
    }

    /**
     * Only using P rn for ANGLE_PID
     */
    public static final class ANGLE_PID
    {
        public static final double P = .8 * (-1.0 / 80.0);
    }

    /**
     * Drivetrain class will load paths from disk and so takes a little bit of time
     */
    public DriveTrain(TalonSRX rightEncoder, TalonSRX leftEncoder)
    {
        rightMotor = new CANSparkMax(WiringIDs.RIGHT_DRIVE_1, MotorType.kBrushless);
        rightFollower = new CANSparkMax(WiringIDs.RIGHT_DRIVE_2, MotorType.kBrushless);

        leftMotor = new CANSparkMax(WiringIDs.LEFT_DRIVE_1, MotorType.kBrushless);
        leftFollower = new CANSparkMax(WiringIDs.LEFT_DRIVE_2, MotorType.kBrushless);

        hDriveMotor = new CANSparkMax(WiringIDs.H_DRIVE, MotorType.kBrushless);

        leftClimbMotor = new VictorSPX(WiringIDs.CLIMBER_1);
        rightClimbMotor = new VictorSPX(WiringIDs.CLIMBER_2);
        rightClimbMotor.setInverted(true);

        climber = new Solenoid(WiringIDs.SOLENOID_CLIMBER);

        this.rightEncoder = rightEncoder;
        this.leftEncoder = leftEncoder;

        rightEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        leftEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        leftEncoder.setSensorPhase(true);

        pigeon = new PigeonIMU(WiringIDs.PIGEON_BASE);

        leftMotor.setInverted(true);
        leftFollower.setInverted(true);

        leftFollower.follow(leftMotor);
        rightFollower.follow(rightMotor);

        /**
         * Path controller, can be configured to use different paths after construction.
         * This call loads from disk
         */
        pathFollower = new HotPathFollower(SECOND_ENCODER_TO_REVS, WHEEL_DIAMETER, Paths.TestPath1.Left,
                Paths.TestPath1.Right);
        pathFollower.ConfigAngleP(ANGLE_PID.P);
        pathFollower.ConfigPosPIDVA(POS_PIDVA.P, POS_PIDVA.I, POS_PIDVA.D, POS_PIDVA.V, POS_PIDVA.A);
    }

    public void loadPath(String leftPathFile, String rightPathFile)
    {
        pathFollower.LoadPath(leftPathFile, rightPathFile);
    }

    /**
     * Control the path follower, should be called on the same period as the
     * profile's time step
     * 
     * @return whether the path is complete
     */
    public boolean FollowPath()
    {
        double heading = xyz_dps[0];
        HotPathFollower.Output pathOutput = pathFollower.FollowNextPoint(-leftEncoderValue, -leftEncoderValue, heading);

        rightMotor.set(pathOutput.Left);
        leftMotor.set(pathOutput.Right);

        return (pathFollower.GetState() == State.Complete);
    }

    public void getYaw()
    {
        pigeon.getYawPitchRoll(xyz_dps);
        currentYaw = -1.0 * Math.toRadians(xyz_dps[0]);
    }

    public void getSingleRotationYaw()
    {
        double rotations;
        pigeon.getYawPitchRoll(xyz_dps);
        singleRotationYaw = -1.0 * Math.toRadians(xyz_dps[0]);
        rotations = singleRotationYaw % (2 * Math.PI);
        singleRotationYaw = (2 * Math.PI) * rotations;
    }

    /**
     * Read the sensors into memory
     */
    public void readSensors()
    {
        rightEncoderValue = rightEncoder.getSelectedSensorPosition();
        leftEncoderValue = leftEncoder.getSelectedSensorPosition();
        pigeon.getYawPitchRoll(xyz_dps);
    }

    public double getPitch()
    {
        return xyz_dps[1];
    }

    public static final List<String> LoggerTags = new ArrayList<>(
            Arrays.asList("Drive rightEncoder", "Drive leftEncoder", "Drive currentYaw", "Drive currentPitch",
                    "Drive currentVelocityLeft", "Drive currentVelocityRight"));

    public boolean canseeTarget()
    {
        if (vmotion.canSeeTarget())
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    private boolean slowRight;
    private boolean slowLeft;

    public void slowRightSide(boolean isTrue)
    {
        slowRight = isTrue;
    }

    public void slowLeftSide(boolean isTrue)
    {
        slowLeft = isTrue;
    }

    /**
     * Write to logs and dashboards
     */
    public void writeLogs()
    {
        SmartDashboard.putNumber("Drive rightEncoder", rightEncoderValue);
        SmartDashboard.putNumber("Drive leftEncoder", leftEncoderValue);
        SmartDashboard.putNumber("Drive currentYaw", xyz_dps[0]);
        SmartDashboard.putNumber("Drive currentPitch", xyz_dps[1]);
        SmartDashboard.putNumber("Drive currentVelocityRight", rightEncoder.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Drive currentVelocityLeft", leftEncoder.getSelectedSensorVelocity());

        /*
         * SmartDashboard.putNumber("motorType", leftMotor.getMotorType().value);
         * SmartDashboard.putNumber("motorEncoderConfiguration",
         * leftMotor.getParameterInt(ConfigParameter.kSensorType).get());
         */
        HotLogger.Log("Drive rightEncoder", rightEncoderValue);
        HotLogger.Log("Drive leftEncoder", leftEncoderValue);
        HotLogger.Log("Drive currentYaw", xyz_dps[0]);
        HotLogger.Log("Drive currentVelocityRight", rightEncoder.getSelectedSensorPosition());
        HotLogger.Log("Drive currentVelocityLeft", leftEncoder.getSelectedSensorVelocity());

        vmotion.writeDashBoardVis();
    }

    /**
     * Clear all measured sensor values in memory and zero the pigeon
     */
    public void zeroSensors()
    {
        pigeon.setYaw(0);
        leftEncoder.setSelectedSensorPosition(0);
        rightEncoder.setSelectedSensorPosition(0);
        leftEncoderValue = 0;
        rightEncoderValue = 0;
        xyz_dps = new double[]
        { 0, 0, 0 };
        pathFollower.Reset();
    }

    /**
     * Motor output to zero
     */
    public void zeroMotors()
    {
        rightMotor.set(0);
        leftMotor.set(0);
        hDriveMotor.set(0);
    }

    public boolean turnComplete(double heading)
    {
        leftMotor.set(0.2);
        rightMotor.set(-0.2);
        if (currentYaw > Math.toDegrees(heading))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public boolean turnToReferenceAngle()
    {
        getSingleRotationYaw();
        double referenceAngle;
        vmotion.sendAngle(singleRotationYaw);
        vmotion.selectTarget(1.0);
        referenceAngle = vmotion.getReferenceAngle();
        if ((singleRotationYaw < (referenceAngle + 0.04)) && (singleRotationYaw > (referenceAngle - 0.04)))
        {
            return true;
        }
        else if (currentYaw > (referenceAngle + 0.04))
        {
            leftMotor.set(0.2);
            rightMotor.set(-0.2);
            return false;
        }
        else
        {
            leftMotor.set(-0.2);
            rightMotor.set(0.2);
            return false;
        }
    }

    public boolean lineUp(double pipeline)
    {
        vmotion.setPipeline(pipeline);
        hDriveMotor.set(vmotion.shuffleVisionPID());
        leftMotor.set(vmotion.outputL());
        rightMotor.set(-vmotion.outputR());
        if (vmotion.targetReached(20.0, 1) == true)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    /*
     * public void updateUsb(int pipeline) { vmotion.usbUpdatePipeline(pipeline); }
     * 
     * public void initUsbCam() { vmotion.usbCamInit(); }
     */

    public boolean gyroLineUp(double maxOutput, double targetDistanceStop)
    {
        switch (state)
        {
        case 0:
            vmotion.setPipeline(1);
            this.getSingleRotationYaw();
            vmotion.sendAngle(singleRotationYaw);
            vmotion.getTargetAngle();
            vmotion.setGyroLineUpVars(1.0);
            state++;
            break;
        case 1:
            this.getSingleRotationYaw();
            vmotion.gyroTargetLineUp(singleRotationYaw, maxOutput);
            double hOutput = vmotion.outputGyroH(singleRotationYaw, maxOutput);
            hDriveMotor.set(hOutput);
            leftMotor.set(vmotion.outputGyroL(singleRotationYaw, maxOutput) + (0.15 * hOutput));
            rightMotor.set(-vmotion.outputGyroR(singleRotationYaw, maxOutput));
            break;
        }
        if (vmotion.targetReached(targetDistanceStop, 1) == true)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    /**
     * Manual control, includes deadband
     * 
     * @param turn
     *                    turn offset for motor output
     * @param forward
     *                    primary forward/backwards output
     * @param side
     *                    the hdrive output value
     */

    private void arcadeDrive(TeleopCommandProvider command)
    {
        double hDrive = HDriveOutput(command.HDrive());
        double hDriveCorrect = 0.15 * hDrive * 0;
        rightMotor.set((command.RightDrive() - command.TurnDrive()) * (slowRight ? .5 : 1));
        leftMotor.set((command.LeftDrive() + hDriveCorrect + command.TurnDrive()) * (slowLeft ? .5 : 1));
        hDriveMotor.set(HDriveOutput(hDrive));
    }

    /*
    boolean havingTarget = false;
    int targetCount = 0;
    boolean hadTargetLast = false;
    boolean hasObtainedTarget = false;
    boolean autoAssistLast = false;
    */
    public void Update(TeleopCommandProvider command)
    {
        /*
        if (havingTarget && targetCount < 15)
        {
            targetCount++;
            command.Rumble();
        }
        else
        {
            targetCount = 0;
            havingTarget = false;
        }
        */
        getYaw();
        // (joystick.getStickRX(), -driver.getStickLY(), (driver.getRawAxis(3) -
        // driver.getRawAxis(2)) / 2.0);
        if (!command.steeringAssistActivated())
        {
            System.out.println("NO VISION");
            arcadeDrive(command);
            vmotion.resetVision();
            /*
            hasObtainedTarget = false;
            havingTarget = false;
            */
        }
        else
        {
            System.out.println("VISION");
            /*
            if (!autoAssistLast) hasObtainedTarget = false;
            if (!hasObtainedTarget && vmotion.canSeeTarget())
            {
                havingTarget = true;
                hasObtainedTarget = true;
            }
            */
            /*
            if (!vmotion.canSeeTarget() && Math.abs(command.TurnDrive()) > 0)
            {
                arcadeDrive(command);
            }
            else*/
            {
                VisionMotion.Output assist = vmotion.autoAlign(-xyz_dps[0]);
                rightMotor.set((command.RightDrive() + assist.Right) * (slowRight ? .5 : 1));
                leftMotor.set((command.LeftDrive() + assist.Left) * (slowLeft ? .5 : 1));
            }
            hDriveMotor.set(command.HDrive());
        }

        if (climbDeployed)
        {
            leftClimbMotor.set(ControlMode.PercentOutput, 1.5 * command.LeftDrive());
            rightClimbMotor.set(ControlMode.PercentOutput, 1.5 * command.RightDrive());
        }
        if (allowClimb && command.ClimberDeploy())
        {
            climbDeployed = true;
            climber.set(true);
        }
        else
            climber.set(false);

        //autoAssistLast = command.steeringAssistActivated();
    }

    private int spikeCounter = 0;
    public static final int H_SPIKE_DURATION = 5;
    public static final double H_SPIKE_MAGNITUDE = .4;
    public static final double H_RAMP = .01;

    public double HDriveOutput(double input)
    {
        SmartDashboard.putString("HState", hDriveState.name());
        double output = 0;
        if (input == 0)
        {
            hDriveState = HDriveState.Off;
            hDrivePrevious = 0;
            spikeCounter = 0;
            return 0;
        }

        if (hDriveState == HDriveState.Off)
        {
            hDriveState = HDriveState.Ramping;
            hDrivePrevious = 0;
            spikeCounter = 0;
        }
        if (hDriveState == HDriveState.Spike)
        {
            spikeCounter++;
            if (spikeCounter > H_SPIKE_DURATION)
            {
                hDriveState = HDriveState.Ramping;
                hDrivePrevious = 0;
                spikeCounter = 0;
            }
            else
                output = H_SPIKE_MAGNITUDE * Math.signum(input);
        }
        if (hDriveState == HDriveState.Ramping)
        {
            SmartDashboard.putNumber("ramp", Math.abs(input) - Math.abs(hDrivePrevious));
            if (Math.abs(input) - Math.abs(hDrivePrevious) > H_RAMP)
            {
                System.out.println("RAMPING");
                output = hDrivePrevious + H_RAMP * Math.signum(input);
            }
            else
                output = input;
        }

        if (Math.abs(output) > 1 && hDriveState != HDriveState.Spike)
        {
            output = Math.signum(output) * 1;
        }
        hDrivePrevious = output;
        HotLogger.Log("H_DRIVE", output);
        SmartDashboard.putNumber("AAA HDRIVE", output);
        SmartDashboard.putNumber("AAA HINPUT", input);
        return output;
    }

    /**
     * Configure the Talon to Calibrate once the robot is stable
     */
    @Override
    public void CalibratePigeon()
    {
        pigeon.enterCalibrationMode(CalibrationMode.BootTareGyroAccel);
    }

    /**
     * Whether the current state of the Pigeon is Ready
     */
    @Override
    public boolean PigeonReady()
    {
        return (pigeon.getState() == PigeonState.Ready);
    }

    public void setAllowClimberDeploy(boolean b)
    {
        allowClimb = b;
    }

    public void useBackCamera(boolean b)
    {
        vmotion.useBackCamera(b);
    }
}