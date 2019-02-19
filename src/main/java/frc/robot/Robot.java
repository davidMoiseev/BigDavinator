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

    public int state = 0;

    @Override
    public void robotInit() {
     // driveTrain.zeroSensors();
        }

    @Override
    public void robotPeriodic() {
      driveTrain.readSensors();
      driveTrain.getYaw();
      driveTrain.writeDashBoard();
    }

    @Override
    public void autonomousInit() {
    //driveTrain.zeroSensors();
    driveTrain.DrivetrainConfig();
    }


    
    @Override
    public void autonomousPeriodic() {
      driveTrain.automaticDropH();
        switch(state){
          case 0:
          if(driveTrain.turnComplete(0.0) == true){
            state++;
          }
        break;
          case 1:
            if(driveTrain.gyroLineUp(1, 0.3, 20.0) == true){
              state++;
            }
          break;
         
          case 2:
          driveTrain.allOff();
          break;
        }
       
    SmartDashboard.putNumber("state", state);

    }


    @Override
    public void teleopPeriodic() {
      driveTrain.driveManualH(1.0, 1.0, 1.0, 1.0);
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