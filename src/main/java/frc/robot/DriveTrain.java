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

import org.hotteam67.HotController;
import org.hotteam67.HotLogger;
import org.hotteam67.HotPathFollower;
import org.hotteam67.HotPathFollower.State;

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

    private VictorSPX leftClimber;
    private VictorSPX rightClimber;

    private boolean allowClimberMotors = false;

    private final PigeonIMU pigeon;
    VisionMotion vmotion = new VisionMotion();
    private final TalonSRX rightEncoder;
    private final TalonSRX leftEncoder;
    private double[] xyz_dps = new double[3];
    private double HDriveOutputOld;
    private int Hstate;
    double k;
    double spike;
    double kAlt;

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
        public static final double P = 0;//.75;
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

        leftClimber = new VictorSPX(WiringIDs.CLIMBER_1);
        rightClimber = new VictorSPX(WiringIDs.CLIMBER_2);
        rightClimber.setInverted(true);

        this.rightEncoder = rightEncoder;
        this.leftEncoder = leftEncoder;

        rightEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        leftEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        leftEncoder.setSensorPhase(true);
        ;

        pigeon = new PigeonIMU(WiringIDs.PIGEON_BASE);

        leftMotor.setInverted(true);
        leftFollower.setInverted(true);

        leftFollower.follow(leftMotor);
        rightFollower.follow(rightMotor);

        /**
         * Path controller, can be configured to use different paths after construction.
         * This call loads from disk
         */
        pathFollower = new HotPathFollower(SECOND_ENCODER_TO_REVS, WHEEL_DIAMETER, Paths.TestPath2.Left,
                Paths.TestPath2.Right);
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
        HotPathFollower.Output pathOutput = pathFollower.FollowNextPoint(leftEncoderValue, rightEncoderValue, -heading);

        rightMotor.set(pathOutput.Right);
        leftMotor.set(pathOutput.Left);

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
        if (vmotion.canSeeTarget() == 1)
        {
            return true;
        }
        else
        {
            return false;
        }
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

    public void updateUsb(int pipeline)
    {
        vmotion.usbUpdatePipeline(pipeline);
    }

    public void initUsbCam()
    {
        vmotion.usbCamInit();
    }

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

    public void Update(TeleopCommandProvider command)
    {
        // (joystick.getStickRX(), -driver.getStickLY(), (driver.getRawAxis(3) -
        // driver.getRawAxis(2)) / 2.0);

        rightMotor.set(command.RightDrive());
        leftMotor.set(command.LeftDrive() + 0.15 * (HDriveOutput(command.HDrive())));

        if (allowClimberMotors)
        {
            leftClimber.set(ControlMode.PercentOutput, command.LeftDrive());
            rightClimber.set(ControlMode.PercentOutput, command.RightDrive());
        }

        hDriveMotor.set(HDriveOutput(command.HDrive()));
    }

    public double HDriveOutput(double input)
    {
        double HDriveOutput = input;
        HDriveOutputOld = HDriveOutput;
        Hstate = 0;
        k = 0.02;
        spike = 0.3;
        kAlt = 0.5;
        // start up if statements spike in the positive and negative/ or do nothing
        // Negative
        if ((Hstate == 0) && (HDriveOutput - HDriveOutputOld) < 0.0 && HDriveOutputOld == 0.0)
        {
            HDriveOutput = HDriveOutput - spike;
            Hstate++;
        }
        // Nothing
        if ((Hstate == 0) && (HDriveOutput - HDriveOutputOld) == 0.0 && Math.abs(HDriveOutputOld) == 0.0)
        {
            HDriveOutputOld = HDriveOutput;
            Hstate = 0;
        }
        // Positive
        if (((Hstate == 0) && (HDriveOutput - HDriveOutputOld) > 0.0 && Math.abs(HDriveOutputOld) == 0.0))
        {
            HDriveOutput = HDriveOutput + spike;
            Hstate++;
        }
        // once moving, either no change, keep state, positive ramp up or ramp down
        // accordingly

        // if at extremes of 0.5 + where spike should be lesser than 0.3
        if ((Hstate == 1) && Math.abs(HDriveOutput - HDriveOutputOld) > 1.0 && HDriveOutputOld > 0.5 || Hstate == 5)
        {
            if (HDriveOutput > 0.0)
            {
                HDriveOutput = Math.abs(HDriveOutput) - k;
                Hstate = 5;
            }
            if (HDriveOutput < 0.0)
            {
                HDriveOutput = HDriveOutput + k;
                Hstate = 5;
            }
            else if (HDriveOutput == 0.0)
            {
                HDriveOutput = 0.0;
                Hstate = 0;
            }
        }
        // nothing
        if ((Hstate == 1) && (HDriveOutput - HDriveOutputOld) == 0.0 && Math.abs(HDriveOutputOld) > 0.0)
        {
            HDriveOutput = HDriveOutputOld;
            Hstate = 1;
        }
        // ramp up
        if ((Hstate == 1) && Math.abs(HDriveOutput - HDriveOutputOld) > 0.0 && Math.abs(HDriveOutputOld) > 0.0)
        {
            if (HDriveOutput > 0.0)
            {
                HDriveOutput = Math.abs(HDriveOutput) + k;
                Hstate = 1;
            }
            else
            {
                HDriveOutput = HDriveOutput - k;
                Hstate = 1;
            }
        }
        // ramp down
        if ((Hstate == 1) && (HDriveOutput - HDriveOutputOld) < 0.0 && Math.abs(HDriveOutputOld) > 0.0)
        {
            if (HDriveOutput > 0.0)
            {
                HDriveOutput = Math.abs(HDriveOutput) + k;
                Hstate = 0;
            }
            else
            {
                HDriveOutput = HDriveOutput - k;
                Hstate = 0;
            }
        }
        // once moving, either no change, keep state, negative ramp up or ramp down
        // accordingly
        return HDriveOutput;
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

    public void SetAllowClimberMotors(boolean allowed)
    {
        allowClimberMotors = allowed;
    }
}