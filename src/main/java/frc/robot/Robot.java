
package frc.robot;

import javax.naming.directory.DirContext;

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
  SweetTurn sweetTurn = new SweetTurn();

  Arm arm = new Arm();
  HotSticks Driver = new HotSticks(0);

  public double target = 0;
  public double is = 0;
  public double position;

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
    sweetTurn.SweetTurnReset();
    arm.Config();
    double position = 0;
  }

  @Override
  public void autonomousPeriodic() {
    double loop = 0;

    
    driveTrain.writeDashBoard();

  }

  @Override
  public void teleopPeriodic() {

    Driver.SetDeadBandLY(0.1);
    Driver.SetDeadBandRX(0.1);
    double turn = Driver.StickRX();
    double foward = Driver.StickLY();

    driveTrain.arcadeDrive(foward * 0.5, turn * 0.5);

    target = target + turn;

    // sweetTurn.SweetTurnFinished(target, 10, 0.1);

    driveTrain.readSensors();
    driveTrain.writeDashBoard();

  }

  @Override
  public void teleopInit() {
    Driver.SetDeadBandLY(0.1);
    Driver.SetDeadBandRX(0.1);
    driveTrain.zeroSensors();
    double target = 0;
    arm.Config();

  }

  @Override
  public void testPeriodic() {

  }
}