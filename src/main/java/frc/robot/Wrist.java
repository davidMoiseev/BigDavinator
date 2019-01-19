/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * Add your docs here.
 */
public class Wrist {

  public static final int arm = 1;
  WPI_TalonSRX Wrist = new WPI_TalonSRX(arm);
  

  void SetPos(double pos) {
    Wrist.set(ControlMode.MotionMagic, pos);
  }

  double GetPos() {   
    double pos = Wrist.getSelectedSensorPosition();
    return pos;
  }
  
  void wristConfig() {
    Wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,3, 100);
    Wrist.setSensorPhase(true);
		Wrist.setInverted(false);
    Wrist.selectProfileSlot(1, 3);
    Wrist.configNominalOutputForward(0, 100);
		Wrist.configNominalOutputReverse(0, 100);
		Wrist.configPeakOutputForward(1, 100);
		Wrist.configPeakOutputReverse(-1, 100);
    Wrist.configMotionCruiseVelocity(15000, 100);
    Wrist.configMotionAcceleration(6000, 100);
  }
}
