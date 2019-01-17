
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {

    String deployPath = Filesystem.getDeployDirectory().getAbsolutePath();

    DriveTrain driveTrain = new DriveTrain();
    
    Joystick stickDrive = new Joystick(0);

    @Override
    public void robotInit() {
    	driveTrain.zeroSensors();
    }

    @Override
    public void robotPeriodic() {
      
      driveTrain.readSensors();
    }

    @Override
    public void autonomousInit() {
      driveTrain.zeroSensors();
    }
    
    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopPeriodic() {
      double forward = stickDrive.getRawAxis(1);
      double turn = stickDrive.getRawAxis(4);

      driveTrain.arcadeDrive((Math.abs(forward) < .05 ? 0 : forward ), (Math.abs(turn) < .05 ? 0 : turn ));
    }
    
    @Override
    public void teleopInit() {
    	driveTrain.zeroSensors();
    }

    @Override
    public void testPeriodic() {
    	
    }
}