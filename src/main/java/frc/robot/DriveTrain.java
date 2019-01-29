package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain {
	
    public static final int TALON_LEFT = 7;
    public static final int TALON_PIGEON = 6;
    public static final int TALON_RIGHT = 5;
    


    WPI_TalonSRX leftTalon = new WPI_TalonSRX(TALON_LEFT);
    WPI_TalonSRX rightTalon = new WPI_TalonSRX(TALON_RIGHT);
    WPI_TalonSRX pigeonTalon = new WPI_TalonSRX(TALON_PIGEON);
    
    public PigeonIMU pigeon = new PigeonIMU(pigeonTalon);
    private double speed;
	private double leftEncoder;
	private double rightEncoder;
	private double[] xyz_dps = new double [3];
	private double currentYaw = 0;
    private double prev;
    private double now;

    public DriveTrain() {
        rightTalon.setSensorPhase(true);
        rightTalon.setInverted(true);
        
        leftTalon.selectProfileSlot(0, 0);
        rightTalon.selectProfileSlot(0, 0);
        
        leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        
        this.zeroSensors();
        
        leftTalon.set(ControlMode.PercentOutput, 0.0);
        rightTalon.set(ControlMode.PercentOutput, 0.0);
	}
		
	
	public void zeroSensors() {
		leftTalon.setSelectedSensorPosition(0, 0, 0); 
        rightTalon.setSelectedSensorPosition(0, 0, 0); 
        pigeon.setYaw(0, 0);        
	}
	
	public void readSensors() {
        leftEncoder = leftTalon.getSelectedSensorPosition(0);
        rightEncoder = rightTalon.getSelectedSensorPosition(0);    
        pigeon.getYawPitchRoll(xyz_dps);
        currentYaw = -1.0 * xyz_dps[0];
	}
	
	public void allOff() {
        leftTalon.set(ControlMode.PercentOutput, 0.0);
        rightTalon.set(ControlMode.PercentOutput, 0.0);
	}
	
	public boolean turnComplete(double heading) {
        leftTalon.set(ControlMode.PercentOutput, 0.2);
        rightTalon.set(ControlMode.PercentOutput, -0.2);
		if (currentYaw > heading) {
			return true;
		} else {
			return false;
		}
	}
	
	public boolean driveStraight(double distance) {
        leftTalon.set(ControlMode.PercentOutput, 0.2);
        rightTalon.set(ControlMode.PercentOutput, 0.2);
		if (rightEncoder > distance) {
			return true;
		} else {
			return false;
		}	
	}
	
    public void writeDashBoard() {
        //now = leftTalon.getSelectedSensorVelocity();

        if (now > speed) {
            speed = now;
        } 
        SmartDashboard.putNumber("leftEncoder", leftEncoder);
        SmartDashboard.putNumber("rightEncoder", rightEncoder);
        SmartDashboard.putNumber("currentYaw", currentYaw);
        SmartDashboard.putNumber("Speed", speed);
    } 
    
    public void arcadeDrive(double f, double t)
    {
        leftTalon.set(ControlMode.PercentOutput, f + t);
        rightTalon.set(ControlMode.PercentOutput, f - t);
    }
}