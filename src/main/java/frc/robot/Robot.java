package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
    DriveTrain driveTrain = new DriveTrain();
    
    Joystick stickDrive = new Joystick(0);

    @Override
    public void robotInit() {
      driveTrain.zeroSensors();
        }

    @Override
    public void robotPeriodic() {
      driveTrain.readSensors();
      driveTrain.writeDashBoard();
    }

    @Override
    public void autonomousInit() {
    driveTrain.zeroSensors();
    driveTrain.DrivetrainConfig();
    }


    
    @Override
    public void autonomousPeriodic() {
    //driveTrain.shuffleVision(); 
    driveTrain.shuffleVisionPID();
    

    }


    @Override
    public void teleopPeriodic() {
      driveTrain.driveManualH(1, 1, 1, 1);


      //driveTrain.arcadeDrive((Math.abs(forward) < .05 ? 0 : forward ), (Math.abs(turn) < .05 ? 0 : turn ));

      
    }
    
    @Override
    public void teleopInit() {
    driveTrain.zeroSensors();
    driveTrain.DrivetrainConfig();
    }

    @Override
    public void testPeriodic() {
    	
    }
}