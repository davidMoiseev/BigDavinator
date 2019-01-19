
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
    }


    
    @Override
    public void autonomousPeriodic() {
      if (Driver.ButtonA() == true) {
            arm.setPosition(10000);
          }
          if (Driver.ButtonB() == true) {
            arm.setPosition(0);
          }
          if (Driver.ButtonX() == true) {
            arm.setPosition(5000);
          }
          
          SmartDashboard.putNumber("Pos", arm.GetPosition());
      }
    



  @Override
  public void teleopPeriodic() {
    target = target + Driver.StickRY();

    arm.setPosition(target);
    SmartDashboard.putNumber("Targ", target * 10);
     SmartDashboard.putNumber("Pos", arm.GetPosition());
    
  }

    @Override
    public void teleopInit() {
      driveTrain.zeroSensors();
      arm.Config();
    }

    @Override
  public void testPeriodic() {
       
      
    }
}