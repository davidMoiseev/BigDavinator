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
    public static final double WHEEL_DIAMETER = 4.0 * 2.54;
    public static final int TICKS_PER_REVOLUTION = 1;

    public static final double ENCODER_TO_REVS = (50.0 / 12.0) * (42.0 / 24.0);
    public static final double SECOND_ENCODER_TO_REVS = (42.0 / 24.0);

    // Recorded max velocity: 3000 units per 100 ms
    // 21,080.986
    public static final double TICKS_PER_METER = ENCODER_TO_REVS * Math.PI * WHEEL_DIAMETER;

    public static final double MAX_VELOCITY_TICKS = 0;// 5800;
    // Max velocity in m/s
    public static final double MAX_VELOCITY = (MAX_VELOCITY_TICKS * TICKS_PER_METER) / 60.0; // 1.4231;

    /**
     * Primary motor controllers
     */
    private final CANSparkMax rightMotor;
    private final CANEncoder rightEncoder;
    private final CANSparkMax leftMotor;
    private final CANEncoder leftEncoder;

    /**
     * Following motor controllers
     */
    private final CANSparkMax rightFollower;
    private final CANSparkMax leftFollower;

    private final CANSparkMax hDriveMotor;
    private final Solenoid hDriveSolenoid;

    private final PigeonIMU pigeon;

    private double rightEncoderValue;
    private double leftEncoderValue;
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
        rightMotor = new CANSparkMax(WiringIDs.LEFT_DRIVE_1, MotorType.kBrushless);
        rightFollower = new CANSparkMax(WiringIDs.LEFT_DRIVE_2, MotorType.kBrushless);

        leftMotor = new CANSparkMax(WiringIDs.RIGHT_DRIVE_1, MotorType.kBrushless);
        leftFollower = new CANSparkMax(WiringIDs.RIGHT_DRIVE_2, MotorType.kBrushless);

        hDriveMotor = new CANSparkMax(WiringIDs.H_DRIVE, MotorType.kBrushless);
        hDriveSolenoid = new Solenoid(WiringIDs.SOLENOID_H_DRIVE);

        rightEncoder = rightMotor.getEncoder();
        leftEncoder = leftMotor.getEncoder();

        pigeon = new PigeonIMU(WiringIDs.PIGEON_BASE);

        leftMotor.setInverted(true);

        leftFollower.follow(leftMotor);
        rightFollower.follow(rightMotor);

        pathFollower = new HotPathFollower(ENCODER_TO_REVS, WHEEL_DIAMETER, Paths.testLeft, Paths.testRight);
        pathFollower.ConfigAngleP(ANGLE_PID.P);
        pathFollower.ConfigPosPIDVA(POS_PIDVA.P, POS_PIDVA.I, POS_PIDVA.D, POS_PIDVA.V, POS_PIDVA.A);
    }

    public boolean FollowPath()
    {
        double heading = xyz_dps[0];
        HotPathFollower.Output pathOutput = pathFollower.FollowNextPoint(leftEncoderValue, rightEncoderValue, -heading);

        rightMotor.set(pathOutput.Right);
        leftMotor.set(pathOutput.Left);

        return (pathFollower.GetState() == State.Complete);
    }

    public void readSensors()
    {
        rightEncoderValue = rightEncoder.getPosition() - leftEncoderLastValue;
        leftEncoderValue = leftEncoder.getPosition() - rightEncoderLastValue;
        pigeon.getYawPitchRoll(xyz_dps);
    }

    public void writeLogs()
    {
        SmartDashboard.putNumber("rightEncoder", rightEncoderValue);
        SmartDashboard.putNumber("leftEncoder", leftEncoderValue);
        SmartDashboard.putNumber("currentYaw", xyz_dps[0]);
        SmartDashboard.putNumber("currentVelocityRight", rightEncoder.getVelocity());
        SmartDashboard.putNumber("currentVelocityLeft", leftEncoder.getVelocity());

        HotLogger.Log("rightEncoder", rightEncoderValue);
        HotLogger.Log("leftEncoder", leftEncoderValue);
        HotLogger.Log("currentYaw", xyz_dps[0]);
        HotLogger.Log("currentVelocityRight", rightEncoder.getVelocity());
        HotLogger.Log("currentVelocityLeft", leftEncoder.getVelocity());
    }

    public void zeroSensors()
    {
        pigeon.setYaw(0);
        leftEncoderLastValue = leftEncoderValue;
        rightEncoderLastValue = rightEncoderValue;
        leftEncoderValue = 0;
        rightEncoderValue = 0;
        xyz_dps = new double[]
        { 0, 0, 0 };
        pathFollower.Reset();
    }

    public void zeroMotors()
    {
        rightMotor.set(0);
        leftMotor.set(0);
    }

    public void arcadeDrive(double turn, double forward, double side)
    {
        double d = .02;
        turn = ((d > turn) && (turn > -d)) ? 0 : turn;
        forward = ((d > forward) && (forward > -d)) ? 0 : -forward;
        side = ((.4 > forward) && (forward > -.4)) ? 0 : side;

        if (side > 0)
            hDriveSolenoid.set(true);
        else
            hDriveSolenoid.set(false);

        rightMotor.set(forward + turn);
        leftMotor.set(forward - turn);
        hDriveMotor.set(side);
    }

    @Override
    public void CalibratePigeon()
    {
        pigeon.enterCalibrationMode(CalibrationMode.BootTareGyroAccel);
    }

    @Override
    public boolean PigeonReady()
    {
        return (pigeon.getState() == PigeonState.Ready);
    }
}