/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/*
driveManual
init 5 motors
*/
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
//Hi Hayley! How are you?
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain {
    public static final int TALON_LF = 7;
    public static final int TALON_PIGEON = 6;
    public static final int TALON_RF = 2;
    public static final int TALON_RB = 3;
    public static final int TALON_LB = 6;
    public static final int TALON_LM = 5;
    public static final int TALON_RM = 4;
    public static final int TALON_H = 1;
    WPI_TalonSRX LFTalon = new WPI_TalonSRX(TALON_LF);
    WPI_TalonSRX RFTalon = new WPI_TalonSRX(TALON_RF);
    WPI_TalonSRX RBTalon = new WPI_TalonSRX(TALON_RB);
    WPI_TalonSRX LBTalon = new WPI_TalonSRX(TALON_LB);
    WPI_TalonSRX HTalon = new WPI_TalonSRX(TALON_H);
    WPI_TalonSRX LMTalon = new WPI_TalonSRX(TALON_LM);
    WPI_TalonSRX RMTalon = new WPI_TalonSRX(TALON_RM);
    PigeonIMU pigeon = new PigeonIMU(TALON_PIGEON);

    HotSticks hotDrive = new HotSticks(0);
    HotSticks hotOp = new HotSticks(1);
    Solenoid solenoidH = new Solenoid(0);

    public double turn;
    public double h;
    public double forward;
    public double speedR;
    public double speedL;
    public double speedH;

    //Joystick Mapping
    public double x;
    public double y;

    public boolean isHDown = false;
    public boolean stateA;
    public boolean prevStateA = false;

    public boolean stateStart;
    public boolean prevStateStart = false;

    public double currentError;
    public double previousError;
    public double distanceTraveled;

    private double sped = 0;
    private double psped = 0;
    private double TopSped = 0;

    public double integral = 0;
    public double leftEncoder;
	public double rightEncoder;
    private double[] xyz_dps = new double [3];
    private double currentYaw = 0;
    int stateMoPro = 0;

    public static final double pGain = 0.03;
    public static final double iGain = 0.000008;
    public static final double dGain = 0.0;




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

        RMTalon.set(ControlMode.Follower, RFTalon.getDeviceID());
        RBTalon.set(ControlMode.Follower, RFTalon.getDeviceID());
        LMTalon.set(ControlMode.Follower, LFTalon.getDeviceID());
        LBTalon.set(ControlMode.Follower, LFTalon.getDeviceID());

    }

    public void dropH(boolean state){
        solenoidH.set(state);
    }

    // public boolean buttonDropH(){
    //     stateA = hotDrive.getButtonA();
    //     if((stateA != prevStateA) && (stateA == false) && (isHDown == false)){
    //         this.dropH(true);
    //         isHDown = true;
    //         prevStateA = stateA;
    //         return true;
    //     }else if((stateA != prevStateA) && (stateA == false) && (isHDown == true)){
    //         this.dropH(false);
    //         isHDown = false;
    //         prevStateA = stateA;
    //         return true;
    //     }else{
    //         prevStateA = stateA;
    //         return false;
    //     }
    // }

    // public boolean dropHOverride(){  
    //     if(this.buttonDropH() == true){
    //        return true; 
    //     }else{
    //         return false;
    //     }
    // }
    
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

    
    public void stopMotors() {
        LFTalon.set(ControlMode.PercentOutput, 0.0);
        RFTalon.set(ControlMode.PercentOutput, 0.0);
   }
     
    
        public void readSensors() {
            leftEncoder = LFTalon.getSelectedSensorPosition();
            rightEncoder = RFTalon.getSelectedSensorPosition(); 
            pigeon.getYawPitchRoll(xyz_dps);
            currentYaw = -1.0 * xyz_dps[0];
        }
        
    	public void zeroGyro() {
            pigeon.setYaw(0);
       }
       public void zeroEncoders() {
        LFTalon.setSelectedSensorPosition(0);   
        RFTalon.setSelectedSensorPosition(0); 
        }

    public double findProportional(double targetHeading){
        double error = xyz_dps[0] - targetHeading;
        return error;
    }

    public double findDerivative(double targetHeading) {
    currentError = this.findProportional(targetHeading);
    double derivative = 0;
    return derivative;
    }

    public double findIntegral(double targetHeading) {
    integral = integral + this.findProportional(targetHeading);
    return integral;
    }


    public boolean driveStraight(double distance, double speed) {
    readSensors();
    distanceTraveled = (leftEncoder + rightEncoder) / 2;
    double Lspeed = speed + (findProportional(0) * pGain) + (findIntegral(0) * iGain);
    double Rspeed = speed - (findProportional(0) * pGain) - (findIntegral(0) * iGain);

     if(distanceTraveled < distance) {
         LFTalon.set(ControlMode.PercentOutput, Lspeed);
         RFTalon.set(ControlMode.PercentOutput, -Rspeed);
         return false;
     }else {

         RFTalon.set(ControlMode.PercentOutput, (0.0));
         return true;
     }
    }
   
     public void turn(double heading) {
	    	double error = -(heading - xyz_dps[0]);
        
	    	double speed = error * .011;

	    	LFTalon.set(ControlMode.PercentOutput, speed);
         RFTalon.set(ControlMode.PercentOutput, speed);
         }

    



   public boolean driveStraightMagically(double distance, int speed, int acc, double kF, double kP, double kI, double kD){
        readSensors();
        distanceTraveled = (leftEncoder + rightEncoder) / 2;
        
        LFTalon.config_kF(0, kF, 20);
        LFTalon.config_kP(0, kP, 20);
        LFTalon.config_kI(0, kI, 20);
        LFTalon.config_kD(0, kD, 20);
        RFTalon.config_kF(0, kF, 20);
        RFTalon.config_kP(0, kP, 20);
        RFTalon.config_kI(0, kI, 20);
        RFTalon.config_kD(0, kD, 20);
        
        RFTalon.configMotionCruiseVelocity(speed, 20);
        RFTalon.configMotionAcceleration(acc, 20);
        LFTalon.configMotionCruiseVelocity(speed, 20);
        LFTalon.configMotionAcceleration(acc, 20);

        if(Math.abs(distanceTraveled) < Math.abs(distance + 5520)) {
            LFTalon.set(ControlMode.MotionMagic, -distance);
            RFTalon.set(ControlMode.MotionMagic, distance);
        return false;
         }else {
            RFTalon.set(ControlMode.PercentOutput, (0.0));
            return true;
        }

    }


    public void writeDashboard(){
        SmartDashboard.putNumber("turn", turn);
        SmartDashboard.putNumber("forward", forward);
        SmartDashboard.putNumber("speedL", speedL);
        SmartDashboard.putNumber("speedR", speedR);
        SmartDashboard.putNumber("JoystickL", hotDrive.getStickLY());
        SmartDashboard.putNumber("JoystickR", hotDrive.getStickRX());
        SmartDashboard.putNumber("yaw", currentYaw);
        SmartDashboard.putNumber("Power LFT", LFTalon.getMotorOutputVoltage());
        SmartDashboard.putNumber("Power LBT", LBTalon.getMotorOutputVoltage());

            SmartDashboard.putNumber("leftEncoder", leftEncoder);
            SmartDashboard.putNumber("rightEncoder", rightEncoder);
            // SmartDashboard.putNumber("yaw", xyz_dps[0]);

            sped = LFTalon.getSelectedSensorVelocity();
            if (sped > psped){
               TopSped = sped;
                psped = sped;
            }
            SmartDashboard.putNumber("Top Speed", TopSped);

        }


}
