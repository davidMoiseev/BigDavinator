package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveTrain {

    public static final int TALON_LF = 100;
    public static final int TALON_PIGEON = 200;
    public static final int TALON_RF = 400;
    public static final int TALON_RB = 67;
    public static final int TALON_LB = 33;
    public static final int TALON_H = 3707;


    WPI_TalonSRX LFTalon = new WPI_TalonSRX(TALON_LF);
    WPI_TalonSRX RFTalon = new WPI_TalonSRX(TALON_RF);
    WPI_TalonSRX RBTalon = new WPI_TalonSRX(TALON_RB);
    WPI_TalonSRX LBTalon = new WPI_TalonSRX(TALON_LB);
    WPI_TalonSRX HTalon = new WPI_TalonSRX(TALON_H);
    public PigeonIMU pigeon = new PigeonIMU(TALON_PIGEON);

    Joystick joystick;
    
	private double leftEncoder;
	private double rightEncoder;
	private double[] xyz_dps = new double [3];
	private double currentYaw = 0;
    
    public DriveTrain() {

        LFTalon.setSensorPhase(true);
        LBTalon.setSensorPhase(true);
        LFTalon.setInverted(true);
        LBTalon.setInverted(true);

        
        // leftTalon.selectProfileSlot(0, 0);
        // rightTalon.selectProfileSlot(0, 0);
        
        LFTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        RFTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        LBTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        RBTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        HTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        
        LFTalon.setSelectedSensorPosition(0); 
        RFTalon.setSelectedSensorPosition(0);
        LBTalon.setSelectedSensorPosition(0); 
        RBTalon.setSelectedSensorPosition(0);
        HTalon.setSelectedSensorPosition(0);
        
        LFTalon.set(ControlMode.PercentOutput, 0.0);
        RFTalon.set(ControlMode.PercentOutput, 0.0);
        LBTalon.set(ControlMode.PercentOutput, 0.0);
        RBTalon.set(ControlMode.PercentOutput, 0.0);
        HTalon.set(ControlMode.PercentOutput, 0.0);
        

        pigeon.setYaw(0);
        
        
        LFTalon.selectProfileSlot(0, 0);
        RFTalon.selectProfileSlot(0, 0);
        LBTalon.selectProfileSlot(0, 0);
        RBTalon.selectProfileSlot(0, 0);
        
        this.zeroSensors();
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
        SmartDashboard.putNumber("leftEncoder", leftEncoder);
        SmartDashboard.putNumber("rightEncoder", rightEncoder);
        SmartDashboard.putNumber("currentYaw", currentYaw);
    } 
    
    public void arcadeDrive(double f, double t)
    {
        leftTalon.set(ControlMode.PercentOutput, f + t);
        rightTalon.set(ControlMode.PercentOutput, f - t);
    }
}