package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import org.hotteam67.HotLogger;
import org.hotteam67.HotPathFollower;
import org.hotteam67.HotPathFollower.State;

public class DriveTrain
{

    public static final double ALLOWED_ERROR_POSITION = 1000;
    public static final double ALLOWED_ERROR_HEADING = .5;

    public static final double WHEEL_DIAMETER = 0.05436;
    public static final int TICKS_PER_REVOLUTION = 3600;
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

    private final WPI_TalonSRX rightTalon = new WPI_TalonSRX(TALON_LEFT);
    private final WPI_TalonSRX leftTalon = new WPI_TalonSRX(TALON_RIGHT);
    private final WPI_TalonSRX pigeonTalon = new WPI_TalonSRX(TALON_PIGEON);
    private final PigeonIMU pigeon = new PigeonIMU(pigeonTalon);

    private int rightEncoder;
    private int leftEncoder;
    private double[] xyz_dps = new double[3];

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
        rightTalon.configFactoryDefault();
        leftTalon.configFactoryDefault();

        leftTalon.setInverted(true);
        rightTalon.setSensorPhase(true);
        leftTalon.setSensorPhase(true);

        rightTalon.selectProfileSlot(0, 0);
        leftTalon.selectProfileSlot(0, 0);

        rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

        rightTalon.setSelectedSensorPosition(0, 0, 0);
        leftTalon.setSelectedSensorPosition(0, 0, 0);

        rightTalon.set(ControlMode.PercentOutput, 0.0);
        leftTalon.set(ControlMode.PercentOutput, 0.0);

        rightTalon.setNeutralMode(NeutralMode.Brake);
        leftTalon.setNeutralMode(NeutralMode.Brake);

        pathFollower = new HotPathFollower(TICKS_PER_REVOLUTION, WHEEL_DIAMETER, Paths.testLeft, Paths.testRight);
        pathFollower.ConfigAnglePID(ANGLE_PID.P);
        pathFollower.ConfigPosPIDVA(POS_PIDVA.P, POS_PIDVA.I, POS_PIDVA.D, POS_PIDVA.V, POS_PIDVA.A);
    }

    public boolean FollowPath()
    {
        double heading = xyz_dps[0];
        HotPathFollower.Output pathOutput = pathFollower.FollowNextPoint(leftEncoder, rightEncoder, -heading);

        rightTalon.set(ControlMode.PercentOutput, pathOutput.Right);
        leftTalon.set(ControlMode.PercentOutput, pathOutput.Left);

        return (pathFollower.GetState() == State.Complete);
    }

    public void readSensors()
    {
        rightEncoder = rightTalon.getSelectedSensorPosition(0);
        leftEncoder = leftTalon.getSelectedSensorPosition(0);
        pigeon.getYawPitchRoll(xyz_dps);
    }

    public void writeLogs()
    {
        HotLogger.Log("rightEncoder", rightEncoder);
        HotLogger.Log("leftEncoder", leftEncoder);
        HotLogger.Log("currentYaw", xyz_dps[0]);
        HotLogger.Log("currentVelocityRight", rightTalon.getSelectedSensorVelocity());
        HotLogger.Log("currentVelocityLeft", leftTalon.getSelectedSensorVelocity());
    }

    public void zeroSensors()
    {
        rightTalon.setSelectedSensorPosition(0, 0, 20);
        leftTalon.setSelectedSensorPosition(0, 0, 20);
        pigeon.setYaw(0);
        leftEncoder = 0;
        rightEncoder = 0;
        xyz_dps = new double[]
        { 0, 0, 0 };
        pathFollower.Reset();
    }

    public void zeroTalons()
    {
        rightTalon.set(ControlMode.PercentOutput, 0);
        leftTalon.set(ControlMode.PercentOutput, 0);
    }

    public void arcadeDrive(double x, double y)
    {
        double d = .02;
        x = ((d > x) && (x > -d)) ? 0 : x;
        y = ((d > y) && (y > -d)) ? 0 : -y;

        rightTalon.set(ControlMode.PercentOutput, y + x);
        leftTalon.set(ControlMode.PercentOutput, y - x);
    }
}