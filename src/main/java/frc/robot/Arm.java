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

/**
 * Add your docs here.
 */
public class Arm {
  public static final int arm = 7;
  public static final int armfollow = 5;
  private static final int UpperLimit = 5000;
  private static final int LowerLimit = -5000;
  private double Position;
  WPI_TalonSRX Arm = new WPI_TalonSRX(arm);
  WPI_TalonSRX Follow = new WPI_TalonSRX(armfollow);
  

  void setPosition(double pos) {

    //Arm.set(ControlMode.PercentOutput, pos);
    Arm.set(ControlMode.MotionMagic, pos);

  }

  void manual(double power) {
    Arm.set(ControlMode.PercentOutput, power);
  }

  void ReadSensor() {
    Position = Arm.getSelectedSensorPosition();
  }

  double GetPosition() {
    return Position;
  }


  void Reconfig() {
    Arm.configFactoryDefault();
    Config();
  }
  void Config(){
    Arm.configFactoryDefault();
    Follow.configFactoryDefault();
    Arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,100);
    Arm.setSelectedSensorPosition(0);
    Arm.setSensorPhase(true);
    Arm.setInverted(false);
    
    Arm.selectProfileSlot(0, 0);
    Arm.configNominalOutputForward(0, 100);
		Arm.configNominalOutputReverse(0, 100);
		Arm.configPeakOutputForward(1, 100);
    Arm.configPeakOutputReverse(-1, 100);
    Arm.configForwardSoftLimitThreshold(UpperLimit);
    Arm.configReverseSoftLimitThreshold(LowerLimit);
    Arm.config_kP(0, 0.05);  
    Arm.config_kI(0, 0);
    Arm.config_kD(0, 0);
    Arm.config_kF(0, 0.163);
    Arm.configMotionCruiseVelocity(5000, 100);
    Arm.configMotionAcceleration(2500, 100);
    Follow.setInverted(false);
    Follow.set(ControlMode.Follower, Arm.getDeviceID());
    
  }

}
