package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SensorType;
import com.revrobotics.CANSparkMaxLowLevel.ConfigParameter;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.hotteam67.HotLogger;
import org.hotteam67.HotPathFollower;
import org.hotteam67.HotPathFollower.State;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private final PigeonIMU pigeon;

    private final TalonSRX rightEncoder;
    private final TalonSRX leftEncoder;
    private double[] xyz_dps = new double[3];

    // values without offset
    private double leftEncoderValue = 0;
    private double rightEncoderValue = 0;

    private final HotPathFollower pathFollower;

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

        rightMotor.set(pathOutput.Left);
        leftMotor.set(pathOutput.Right);

        return (pathFollower.GetState() == State.Complete);
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

    /**
     * Write to logs and dashboards
     */
    public void writeLogs()
    {
        SmartDashboard.putNumber("rightEncoder", rightEncoderValue);
        SmartDashboard.putNumber("leftEncoder", leftEncoderValue);
        SmartDashboard.putNumber("currentYaw", xyz_dps[0]);
        SmartDashboard.putNumber("currentVelocityRight", rightEncoder.getSelectedSensorVelocity());
        SmartDashboard.putNumber("currentVelocityLeft", leftEncoder.getSelectedSensorVelocity());

        /*
         * SmartDashboard.putNumber("motorType", leftMotor.getMotorType().value);
         * SmartDashboard.putNumber("motorEncoderConfiguration",
         * leftMotor.getParameterInt(ConfigParameter.kSensorType).get());
         */
        HotLogger.Log("rightEncoder", rightEncoderValue);
        HotLogger.Log("leftEncoder", leftEncoderValue);
        HotLogger.Log("currentYaw", xyz_dps[0]);
        HotLogger.Log("currentVelocityRight", rightEncoder.getSelectedSensorPosition());
        HotLogger.Log("currentVelocityLeft", leftEncoder.getSelectedSensorVelocity());
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
    public void arcadeDrive(double turn, double forward, double side)
    {
        hDriveMotor.set(side);
        rightMotor.set(forward - turn);
        leftMotor.set(forward + turn);
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
}