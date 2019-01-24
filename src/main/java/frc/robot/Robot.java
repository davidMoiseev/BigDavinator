/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix.sensors.PigeonIMU;
//import edu.wpi.first.wpilibj.RobotBase;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {

    private CANSparkMax m_motor;

  private static final int deviceID = 5;
  public HotSticks m_stick;
  public CANEncoder m_encoder;
  

  @Override
  public void robotInit() {
    m_stick = new HotSticks(0);
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    m_encoder = m_motor.getEncoder();
  }

  @Override
  public void autonomousInit() {
   
  }

  @Override
  public void autonomousPeriodic() {
  
  }

  @Override
  public void teleopPeriodic() {
    m_motor.set(m_stick.getStickLY());
    SmartDashboard.putNumber("Voltage", m_motor.getBusVoltage());
    SmartDashboard.putNumber("Temperature", m_motor.getMotorTemperature());
    SmartDashboard.putNumber("Output", m_motor.getAppliedOutput());
    SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
    SmartDashboard.putNumber("Encoder Velocity", m_encoder.getVelocity());
  }

  @Override
  public void testPeriodic() {
  }
}
