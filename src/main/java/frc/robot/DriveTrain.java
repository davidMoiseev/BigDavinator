package frc.robot;

import java.io.File;
import java.io.IOException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import org.hotteam67.HotLogger;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class DriveTrain {

    public static final double ALLOWED_ERROR_POSITION = 1000;
    public static final double ALLOWED_ERROR_HEADING = .5;

    public static final double WHEEL_DIAMETER = 0.05436;
    public static final double TICKS_PER_REVOLUTION = 3600.0;
    // Recorded max velocity: 3000 units per 100 ms
    // 21,080.986
    public static final double TICKS_PER_METER = (TICKS_PER_REVOLUTION / (Math.PI * WHEEL_DIAMETER));

    // Ticks per 100 ms, as read from getSelectedSensorVelocity(0)
    public static final double MAX_VELOCITY_TICKS = 5800;
    // Max velocity in m/s
    public static final double MAX_VELOCITY = MAX_VELOCITY_TICKS * 10 / TICKS_PER_METER; // 1.4231;
    public static final double MAX_ACCEL = 5.821;

    public static final int TALON_LEFT = 1;
    public static final int TALON_PIGEON = 2;
    public static final int TALON_RIGHT = 4;

    WPI_TalonSRX leftTalon = new WPI_TalonSRX(TALON_LEFT);
    WPI_TalonSRX rightTalon = new WPI_TalonSRX(TALON_RIGHT);
    WPI_TalonSRX pigeonTalon = new WPI_TalonSRX(TALON_PIGEON);

    PigeonIMU pigeon = new PigeonIMU(pigeonTalon);

    private double leftEncoder;
    private double rightEncoder;
    private double[] xyz_dps = new double[3];

    /*
     * Motion Profiling Constants
     */
    public static final class POS_PIDVA {
        public static final double P = .8;
        public static final double I = 0;
        public static final double D = 0;
        public static final double V = 1.0 / MAX_VELOCITY; // Velocity feed forward
        public static final double A = 0; // Acceleration gain
    }

    public static final class ANGLE_PID {
        public static final double P = .8 * (-1.0 / 80.0);
        public static final double I = 0;
        public static final double D = 0;
    }

    // Controllers
    private EncoderFollower leftEncoderFollower;
    private EncoderFollower rightEncoderFollower;

    public DriveTrain() {
        leftTalon.configFactoryDefault();
        rightTalon.configFactoryDefault();

        leftTalon.setInverted(true);
        leftTalon.setSensorPhase(true);
        rightTalon.setSensorPhase(true);

        leftTalon.selectProfileSlot(0, 0);
        rightTalon.selectProfileSlot(0, 0);

        leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

        leftTalon.setSelectedSensorPosition(0, 0, 0);
        rightTalon.setSelectedSensorPosition(0, 0, 0);

        leftTalon.set(ControlMode.PercentOutput, 0.0);
        rightTalon.set(ControlMode.PercentOutput, 0.0);

        leftTalon.setNeutralMode(NeutralMode.Brake);
        rightTalon.setNeutralMode(NeutralMode.Brake);

        setupMotionProfiling();

        leftEncoderFollower.reset();
        rightEncoderFollower.reset();
    }

    private void setupMotionProfiling() {
        try {
            File left = new File(Filesystem.getDeployDirectory() + Paths.testLeft);
            File right = new File(Filesystem.getDeployDirectory() + Paths.testRight);

            if (!left.exists() || !right.exists())
                throw new IOException();

            leftEncoderFollower = new EncoderFollower(Pathfinder.readFromCSV(left));
            rightEncoderFollower = new EncoderFollower(Pathfinder.readFromCSV(right));
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Failed to load motion profile files");
            return;
        }

        leftEncoderFollower.configureEncoder(0, (int) TICKS_PER_REVOLUTION, WHEEL_DIAMETER);
        rightEncoderFollower.configureEncoder(0, (int) TICKS_PER_REVOLUTION, WHEEL_DIAMETER);

        leftEncoderFollower.configurePIDVA(POS_PIDVA.P, POS_PIDVA.I, POS_PIDVA.D, POS_PIDVA.V, POS_PIDVA.A);
        rightEncoderFollower.configurePIDVA(POS_PIDVA.P, POS_PIDVA.I, POS_PIDVA.D, POS_PIDVA.V, POS_PIDVA.A);
    }

    private int followIterations = 0;
    private Segment lastTargetLeft = null;
    private Segment lastTargetRight = null;
    private double lastTargetHeading = 0;

    public boolean FollowPath() {
        HotLogger.Log("Path Points", followIterations);
        ++followIterations;

        double l = leftEncoderFollower.calculate((int) leftEncoder);
        double r = rightEncoderFollower.calculate((int) rightEncoder);

        try {
            lastTargetLeft = leftEncoderFollower.getSegment();
            lastTargetRight = rightEncoderFollower.getSegment();
            lastTargetHeading = leftEncoderFollower.getHeading();
        } catch (Exception e) {
        }
        double heading = xyz_dps[0];
        double angleError = Pathfinder.boundHalfDegrees(Pathfinder.r2d(lastTargetHeading) - heading);
        double turn = ANGLE_PID.P * angleError;

        if (lastTargetLeft != null && lastTargetRight != null && leftEncoderFollower.isFinished()
                && rightEncoderFollower.isFinished()) {
            double leftError = lastTargetLeft.position - (leftEncoder / TICKS_PER_METER);
            double rightError = lastTargetRight.position - (rightEncoder / TICKS_PER_METER);
            if (Math.abs(leftError) < ALLOWED_ERROR_POSITION && Math.abs(rightError) < ALLOWED_ERROR_POSITION
                    && Math.abs(angleError) < ALLOWED_ERROR_HEADING)
                return true;

            if (l == 0 || r == 0) {

                l = manualPV(leftError, lastTargetLeft.velocity);
                r = manualPV(leftError, lastTargetLeft.velocity);

                return true;
            }
        } else if (leftEncoderFollower.isFinished() && rightEncoderFollower.isFinished())
            return true;

        Log("Left", lastTargetLeft, l);
        Log("Right", lastTargetRight, r);
        HotLogger.Log("Path Heading", lastTargetHeading);
        HotLogger.Log("Heading Error", angleError);
        HotLogger.Log("Turn Output", turn);

        leftTalon.set(ControlMode.PercentOutput, l - turn);
        rightTalon.set(ControlMode.PercentOutput, r + turn);

        return false;
    }

    private double manualPV(double error, double v) {
        return POS_PIDVA.P * error + POS_PIDVA.V * v;
    }

    private void Log(String motorName, Segment s, double output) {
        if (s == null)
            return;
        HotLogger.Log(motorName + " Path Position", s.position);
        HotLogger.Log(motorName + " Path Velocity", s.velocity);
        HotLogger.Log(motorName + " Path Acceleration", s.acceleration);
        HotLogger.Log(motorName + " Path X", s.x);
        HotLogger.Log(motorName + " Path Y", s.y);
        HotLogger.Log(motorName + " Output", output);
    }

    public void readSensors() {
        leftEncoder = leftTalon.getSelectedSensorPosition(0);
        rightEncoder = rightTalon.getSelectedSensorPosition(0);

        pigeon.getYawPitchRoll(xyz_dps);
    }

    public void writeDashBoard() {
        HotLogger.Log("leftEncoder", leftEncoder);
        HotLogger.Log("rightEncoder", rightEncoder);
        HotLogger.Log("currentYaw", xyz_dps[0]);
        HotLogger.Log("currentVelocityLeft", leftTalon.getSelectedSensorVelocity());
        HotLogger.Log("currentVelocityRight", rightTalon.getSelectedSensorVelocity());
        SmartDashboard.putNumber("leftEncoder", leftEncoder);
        SmartDashboard.putNumber("rightEncoder", rightEncoder);
        SmartDashboard.putNumber("currentYaw", xyz_dps[0]);
        SmartDashboard.putNumber("currentVelocityLeft", leftTalon.getSelectedSensorVelocity());
        SmartDashboard.putNumber("currentVelocityRight", rightTalon.getSelectedSensorVelocity());
    }

    public void zeroSensors() {
        leftTalon.setSelectedSensorPosition(0, 0, 20);
        rightTalon.setSelectedSensorPosition(0, 0, 20);
        pigeon.setYaw(0, 0);
    }

    public void zeroTalons() {
        leftTalon.set(ControlMode.PercentOutput, 0);
        rightTalon.set(ControlMode.PercentOutput, 0);
        leftEncoderFollower.reset();
        rightEncoderFollower.reset();
        followIterations = 0;
        lastTargetHeading = 0;
        lastTargetLeft = null;
        lastTargetRight = null;
    }

    public void arcadeDrive(double x, double y) {
        SmartDashboard.putNumber("Turn Input", x);
        SmartDashboard.putNumber("Forward Input", y);
        x = deadband(x, .02);
        y = deadband(-y, .02);
        leftTalon.set(ControlMode.PercentOutput, y, DemandType.ArbitraryFeedForward, -x);
        rightTalon.set(ControlMode.PercentOutput, y, DemandType.ArbitraryFeedForward, x);
    }

    public double deadband(double input, double deadband) {
        if ((deadband > input) && (input > -deadband))
            return 0;
        else
            return input;
    }
}