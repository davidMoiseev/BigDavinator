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

/**
 * Add your docs here.
 */
public class Wrist {

  public static final int arm = 1;
  WPI_TalonSRX Wrist = new WPI_TalonSRX(arm);
  private double Position;

  void SetPos(double pos) {
    Wrist.set(ControlMode.MotionMagic, pos);
  }

  void ReadSensor() {
    double Positon = Wrist.getSelectedSensorPosition();
  }

  double GetPosition() {   
    return Position;
  }
  
  void wristConfig() {
    Wrist.configFactoryDefault();
    Wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,100);
    Wrist.setSelectedSensorPosition(0);
    Wrist.setSensorPhase(true);
    Wrist.setInverted(false);
    Wrist.selectProfileSlot(0, 0);
    Wrist.configNominalOutputForward(0, 100);
		Wrist.configNominalOutputReverse(0, 100);
		Wrist.configPeakOutputForward(1, 100);
    Wrist.configPeakOutputReverse(-1, 100);
    Wrist.config_kP(0, 0.05);
    Wrist.config_kI(0, 0);
    Wrist.config_kD(0, 0);
    Wrist.config_kF(0, 0.163);
    Wrist.configMotionCruiseVelocity(5000, 100);
  }
}
