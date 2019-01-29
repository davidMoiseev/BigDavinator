
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
    Arm arm = new Arm();
    HotSticks Driver = new HotSticks(0);
    
    public double target = 0;
  public double is = 0;
    @Override
    public void robotInit() {
    driveTrain.zeroSensors();
    
    Driver.SetDeadBandRY(0.1);
    }

    @Override
    public void robotPeriodic() {
      arm.ReadSensor();
      driveTrain.readSensors();
    }

    @Override
    public void autonomousInit() {
    driveTrain.zeroSensors();
      arm.Config();
    }


    
    @Override
    public void autonomousPeriodic() {
      if (Driver.ButtonA() == true) {
          arm.setPosition(15000);
      is = 1;  
          }
          if (Driver.ButtonB() == true) {
      arm.setPosition(0);
      is = 2;
          }
          if (Driver.ButtonX() == true) {
      arm.setPosition(-15000);
      is = 3;
          }
          
    SmartDashboard.putNumber("Pos", arm.GetPosition());
    SmartDashboard.putNumber("S", is);
      }
    



  @Override
  public void teleopPeriodic() {
    
    double turn = Driver.StickRX();

    double foward = Driver.StickLY();

    //driveTrain.arcadeDrive(foward * 0.5, turn * 0.5);
    arm.manual(foward);

    driveTrain.readSensors();
    driveTrain.writeDashBoard();
    

  }

    @Override
  public void teleopInit() {
      Driver.SetDeadBandLY(0.1);
      Driver.SetDeadBandRX(0.1); 
      driveTrain.zeroSensors();
      arm.Config();
      
    }
                           
    @Override
  public void testPeriodic() {
       
      
    }
}