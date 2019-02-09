package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class DriveTrain {
	
    public static final int TALON_LF = 7;
    public static final int TALON_RF = 2;
    public static final int TALON_RB = 3;
    public static final int TALON_LB = 6;
    public static final int TALON_LM = 5;
    public static final int TALON_RM = 4;
    public static final int TALON_H = 1;
    public static final int TALON_PIGEON = 6;
    
    // WPI_TalonSRX pigeon = new WPI_TalonSRX(TALON_PIGEON);
    WPI_TalonSRX LFTalon = new WPI_TalonSRX(TALON_LF);
    WPI_TalonSRX RFTalon = new WPI_TalonSRX(TALON_RF);
    WPI_TalonSRX RBTalon = new WPI_TalonSRX(TALON_RB);
    WPI_TalonSRX LBTalon = new WPI_TalonSRX(TALON_LB);
    WPI_TalonSRX HTalon = new WPI_TalonSRX(TALON_H);
    WPI_TalonSRX LMTalon = new WPI_TalonSRX(TALON_LM);
    WPI_TalonSRX RMTalon = new WPI_TalonSRX(TALON_RM);
  
    public PigeonIMU pigeon = new PigeonIMU(TALON_PIGEON);
    VisionMotion vmotion = new VisionMotion();

    HotSticks hotDrive = new HotSticks(0);
    Solenoid solenoid = new Solenoid(0);
    public double turn = hotDrive.getStickRX();
    public double  forward = hotDrive.getStickRY();  
	private double leftEncoder;
	private double rightEncoder;
	private double[] xyz_dps = new double [3];
    private double currentYaw = 0;
    public double h;
    public double speedH;
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
    public double pAngle;
    public double iAngle;
    
    public void DrivetrainConfig(){

        LFTalon.configFactoryDefault();
        RFTalon.configFactoryDefault();
        LMTalon.configFactoryDefault();
        RMTalon.configFactoryDefault();
        LBTalon.configFactoryDefault();
        RBTalon.configFactoryDefault();
 
        LFTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
        RFTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
 
        LFTalon.setSelectedSensorPosition(0);
        RFTalon.setSelectedSensorPosition(0);
        HTalon.setSelectedSensorPosition(0);
 
        LFTalon.selectProfileSlot(0, 0);
        RFTalon.selectProfileSlot(0, 0);
        LFTalon.configNominalOutputForward(0, 100);
        RFTalon.configNominalOutputForward(0, 100);
        LFTalon.configNominalOutputReverse(0, 100);
        RFTalon.configNominalOutputReverse(0, 100);
        LFTalon.configPeakOutputForward(1, 100);
        RFTalon.configPeakOutputForward(1, 100);
        LFTalon.configPeakOutputReverse(-1, 100);
        RFTalon.configPeakOutputReverse(-1, 100);
 
        LFTalon.setSensorPhase(true);
 
        LFTalon.set(ControlMode.PercentOutput, 0.0);
        RFTalon.set(ControlMode.PercentOutput, 0.0);
        HTalon.set(ControlMode.PercentOutput, 0.0);
 
        // LFTalon.setNeutralMode(NeutralMode.Brake);
        // RFTalon.setNeutralMode(NeutralMode.Brake);
        // HTalon.setNeutralMode(NeutralMode.Brake);
 
        pigeon.setYaw(0);
        // this.zeroSensors();
        RMTalon.set(ControlMode.Follower, RFTalon.getDeviceID());
        RBTalon.set(ControlMode.Follower, RFTalon.getDeviceID());
        LMTalon.set(ControlMode.Follower, LFTalon.getDeviceID());
        LBTalon.set(ControlMode.Follower, LFTalon.getDeviceID());
 
    }

    public void dropH(boolean state){
        solenoid.set(state);
        if(state == true) {
            isHDown = true;
        }
        else{
            isHDown = false;
        }
    }

    public void automaticDropH(){
        if(HTalon.getMotorOutputPercent() > 0.05){
            this.dropH(true);
        }else if(HTalon.getMotorOutputPercent() < -0.05){
            this.dropH(true);
        }else{
            this.dropH(false);
        }
    }

    public void zeroSensors(){
        LFTalon.setSelectedSensorPosition(0,0,0);
        RFTalon.setSelectedSensorPosition(0,0,0);
        pigeon.setYaw(0,0);
    }
 
    public void driveManualH(double kFwd, double kTurn, double kSpeed, double kH){
        hotDrive.setDeadBandRY(0.1);
        hotDrive.setDeadBandLX(0.1);
        hotDrive.setDeadBandRX(0.1);
        turn =  hotDrive.getStickLX();
        h = hotDrive.getStickRX();
        forward = hotDrive.getStickRY();
        double speedR = (forward * kFwd) + (turn * kTurn);
        double speedL = -(forward * kFwd) + (turn * kTurn);
        speedH = (h * kH);
        if(speedR > 1){
            speedR = 1;
        }else if(speedL > 1){
            speedL = 1;
        }
        if(hotDrive.getButtonA() == true){
            this.dropH(true);
        }else if(Math.abs(speedH) > 0.1){
            this.dropH(true);
        }else{
            this.dropH(false);
        }
        if((speedR <= 1) && (speedL <= 1) && (speedH <= 1)){
            RFTalon.set(ControlMode.PercentOutput, (speedR * kSpeed));
            LFTalon.set(ControlMode.PercentOutput, (speedL * kSpeed));
            HTalon.set(ControlMode.PercentOutput, (speedH));
        }
    }

	
	public void readSensors() {
        leftEncoder = LFTalon.getSelectedSensorPosition(0);
        rightEncoder = RFTalon.getSelectedSensorPosition(0);    
        pigeon.getYawPitchRoll(xyz_dps);
        currentYaw = -1.0 * xyz_dps[0]; 
	}
	
	public void allOff() {
        LFTalon.set(ControlMode.PercentOutput, 0.0);
        RFTalon.set(ControlMode.PercentOutput, 0.0);
	}
	
	public boolean turnComplete(double heading) {
        LFTalon.set(ControlMode.PercentOutput, 0.2);
        RFTalon.set(ControlMode.PercentOutput, -0.2);
		if (currentYaw > heading) {
			return true;
		} else {
			return false;
		}
    }

    public void turnGyro(double heading){
        if (heading > 0) {
            LFTalon.set(ControlMode.PercentOutput, 0.2);
            RFTalon.set(ControlMode.PercentOutput, -0.2); }
            else if (heading < 0) {
                LFTalon.set(ControlMode.PercentOutput, -0.2);
                RFTalon.set(ControlMode.PercentOutput, 0.2);
            }
            else {
                LFTalon.set(ControlMode.PercentOutput, 0.0);
                RFTalon.set(ControlMode.PercentOutput, 0.0);
            };
    }
    
	public boolean driveStraight(double distance) {
        LFTalon.set(ControlMode.PercentOutput, 0.2);
        RFTalon.set(ControlMode.PercentOutput, 0.2);
		if (rightEncoder > distance) {
			return true;
		} else {
			return false;
		}	
    }
    // public void basicTurnOutput(double speed){
    //     LFTalon.set(ControlMode.PercentOutput, speed);
    //     RFTalon.set(ControlMode.PercentOutput, -speed);
    // }

    // public void basicSideOutput(double speed){
    //     HTalon.set(ControlMode.PercentOutput, speed);
    // }

    public void LRControlVis(){
        LFTalon.set(ControlMode.PercentOutput, vmotion.turnVision());
        RFTalon.set(ControlMode.PercentOutput, -vmotion.turnVision());
    }

    public void HControlVis(){
        HTalon.set(ControlMode.PercentOutput, vmotion.shuffleVisionPID());
    }
    
    public boolean followBall(){
        HTalon.set(ControlMode.PercentOutput, vmotion.shuffleVisionPID());
        LFTalon.set(ControlMode.PercentOutput, vmotion.outputLBall());
        RFTalon.set(ControlMode.PercentOutput, -vmotion.outputRBall());
        if(vmotion.ballHasBeenFollowed() == true){
            return true;
        }else{
            return false;
        }
    }

	public void writeDashBoard() {
        SmartDashboard.putNumber("leftEncoder", leftEncoder);
        SmartDashboard.putNumber("rightEncoder", rightEncoder);
        SmartDashboard.putNumber("currentYaw", currentYaw);
        SmartDashboard.putBoolean("Is h down", isHDown);
        vmotion.writeDashBoardVis();
        SmartDashboard.putNumber("outputL", vmotion.outputLBall());
        SmartDashboard.putNumber("outputR", vmotion.outputRBall());
    } 
    
    public void arcadeDrive(double f, double t)
    {
        LFTalon.set(ControlMode.PercentOutput, f + t);
        RFTalon.set(ControlMode.PercentOutput, f - t);
        
    }
}