/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/*
package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * Add your docs here.
 */
/*public class SuiteBench {
    public class Robot extends TimedRobot {
        public HotSticks m_stick;
        private static final int deviceID = 1;
        private CANSparkMax m_motor;
    }
    {}
          /*
           * deviceID is the CAN ID of the SPARK MAX you are using.
           * Change to match your setup
           *
      
          /*
           * Parameters can be set by calling the appropriate Set method on the CANSparkMax object
           * whose properties you want to change
           * 
           * Set methods will return one of three CANError values which will let you know if the 
           * parameter was successfully set:
           *  CANError.kOk
           *  CANError.kError
           *  CANError.kTimeout
           *
          if(m_motor.setIdleMode(IdleMode.kCoast) != CANError.kOK){
            SmartDashboard.putString("Idle Mode", "Error");
          }
      
          /**
           * Similarly, parameters will have a Get method which allows you to retrieve their values
           * from the controller
           *
          if(m_motor.getIdleMode() == IdleMode.kCoast) {
            SmartDashboard.putString("Idle Mode", "Coast");
          } else {
            SmartDashboard.putString("Idle Mode", "Brake");
          }
      
          // Set ramp rate to 0
          if(m_motor.setRampRate(0) != CANError.kOK) {
            SmartDashboard.putString("Ramp Rate", "Error");
          }
      
          // read back ramp rate value
          SmartDashboard.putNumber("Ramp Rate", m_motor.getRampRate());
      
          m_stick = new HotSticks(0);
        }
       {
          // Set motor output to joystick value
          m_motor.set(m_stick.getY());
          
          // periodically read voltage, temperature, and applied output and publish to SmartDashboard
          SmartDashboard.putNumber("Voltage", m_motor.getBusVoltage());
          SmartDashboard.putNumber("Temperature", m_motor.getMotorTemperature());
          SmartDashboard.putNumber("Output", m_motor.getAppliedOutput());
        }
      }
*/
