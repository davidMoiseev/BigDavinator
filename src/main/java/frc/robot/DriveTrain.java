package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.hotteam67.HotLogger;
import org.hotteam67.HotPathFollower;
import org.hotteam67.HotPathFollower.State;

import frc.robot.constants.ControllerAreaNetworkIdentificationNumbers;

public class DriveTrain
{

    public static final double ALLOWED_ERROR_POSITION = 1000;
    public static final double ALLOWED_ERROR_HEADING = .5;

    public static final double WHEEL_DIAMETER = 0.05436;
    public static final int TICKS_PER_REVOLUTION = 1; //3600;
    // Recorded max velocity: 3000 units per 100 ms
    // 21,080.986
    public static final double TICKS_PER_METER = (TICKS_PER_REVOLUTION / (Math.PI * WHEEL_DIAMETER));

    // Ticks per 100 ms, as read from getSelectedSensorVelocity(0)
    public static final double MAX_VELOCITY_TICKS = 5800;
    // Max velocity in m/s
    public static final double MAX_VELOCITY = MAX_VELOCITY_TICKS * 10 / TICKS_PER_METER; // 1.4231;

    public static final int TALON_LEFT = 4;
    public static final int TALON_PIGEON = 2;
    public static final int TALON_RIGHT = 1;

    private final CANSparkMax rightMotor = new CANSparkMax(ControllerAreaNetworkIdentificationNumbers.LEFT_DRIVE_1,
            MotorType.kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(ControllerAreaNetworkIdentificationNumbers.LEFT_DRIVE_2,
            MotorType.kBrushless);

    private final CANSparkMax leftMotor = new CANSparkMax(ControllerAreaNetworkIdentificationNumbers.RIGHT_DRIVE_1,
            MotorType.kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(ControllerAreaNetworkIdentificationNumbers.RIGHT_DRIVE_2,
            MotorType.kBrushless);

    private final TalonSRX pigeonTalon = new TalonSRX(ControllerAreaNetworkIdentificationNumbers.ELEVATOR_2);
    private final PigeonIMU pigeon = new PigeonIMU(pigeonTalon);

    private double rightEncoder;
    private double leftEncoder;
    private double[] xyz_dps = new double[3];
    private double leftEncoderLastValue = 0;
    private double rightEncoderLastValue = 0;

    private final HotPathFollower pathFollower;

    /*
     * Motion Profiling Constants
     */
    public static final class POS_PIDVA
    {
        public static final double P = .8;
        public static final double I = 0;
        public static final double D = 0;
        public static final double V = 1.0 / MAX_VELOCITY; // Velocity feed forward
        public static final double A = 0; // Acceleration gain
    }

    public static final class ANGLE_PID
    {
        public static final double P = .8 * (-1.0 / 80.0);
        public static final double I = 0;
        public static final double D = 0;
    }

    public DriveTrain()
    {
        /*
         * rightTalon.configFactoryDefault(); leftTalon.configFactoryDefault();
         */

        leftMotor.setInverted(true);
        leftFollower.follow(leftMotor);
        rightFollower.follow(rightMotor);
        /*
         * rightTalon.setSensorPhase(true); leftTalon.setSensorPhase(true);
         * 
         * rightTalon.selectProfileSlot(0, 0); leftTalon.selectProfileSlot(0, 0);
         * 
         * rightTalon.configSelectedFeedbackSensor(FeedbackDevice.
         * CTRE_MagEncoder_Relative, 0, 0);
         * leftTalon.configSelectedFeedbackSensor(FeedbackDevice.
         * CTRE_MagEncoder_Relative, 0, 0);
         * 
         * 
         * rightTalon.setSelectedSensorPosition(0, 0, 0);
         * leftTalon.setSelectedSensorPosition(0, 0, 0);
         * 
         * rightTalon.set(ControlMode.PercentOutput, 0.0);
         * leftTalon.set(ControlMode.PercentOutput, 0.0);
         * 
         * rightTalon.setNeutralMode(NeutralMode.Brake);
         * leftTalon.setNeutralMode(NeutralMode.Brake);
         */

        pathFollower = new HotPathFollower(TICKS_PER_REVOLUTION, WHEEL_DIAMETER, Paths.testLeft, Paths.testRight);
        pathFollower.ConfigAnglePID(ANGLE_PID.P);
        pathFollower.ConfigPosPIDVA(POS_PIDVA.P, POS_PIDVA.I, POS_PIDVA.D, POS_PIDVA.V, POS_PIDVA.A);
    }

    public boolean FollowPath()
    {
        double heading = xyz_dps[0];
        HotPathFollower.Output pathOutput = pathFollower.FollowNextPoint(leftEncoder, rightEncoder, -heading);

        rightMotor.set(pathOutput.Right);
        leftMotor.set(pathOutput.Left);

        return (pathFollower.GetState() == State.Complete);
    }

    public void readSensors()
    {
        rightEncoder = rightMotor.getEncoder().getPosition() - leftEncoderLastValue;
        leftEncoder = leftMotor.getEncoder().getPosition() - rightEncoderLastValue;
        pigeon.getYawPitchRoll(xyz_dps);
    }

    public void writeLogs()
    {
        HotLogger.Log("rightEncoder", rightEncoder);
        HotLogger.Log("leftEncoder", leftEncoder);
        HotLogger.Log("currentYaw", xyz_dps[0]);
        HotLogger.Log("currentVelocityRight", rightMotor.getEncoder().getVelocity());
        HotLogger.Log("currentVelocityLeft", leftMotor.getEncoder().getVelocity());
    }

    public void zeroSensors()
    {
        pigeon.setYaw(0);
        leftEncoderLastValue = leftEncoder;
        rightEncoderLastValue = rightEncoder;
        leftEncoder = 0;
        rightEncoder = 0;
        xyz_dps = new double[]
        { 0, 0, 0 };
        pathFollower.Reset();
    }

    public void zeroTalons()
    {
        rightMotor.set(0);
        leftMotor.set(0);
    }

    public void arcadeDrive(double x, double y)
    {
        double d = .02;
        x = ((d > x) && (x > -d)) ? 0 : x;
        y = ((d > y) && (y > -d)) ? 0 : -y;

        rightMotor.set(y + x);
        leftMotor.set(y - x);
    }
}