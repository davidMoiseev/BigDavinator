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

  public static final int arm = 1;
  public static final int armfollow = 4;
  private static final int UpperLimit = 6767;
  private static final int LowerLimit = 2727;
  private double Position;
  WPI_TalonSRX Arm = new WPI_TalonSRX(arm);
  WPI_TalonSRX Follow = new WPI_TalonSRX(armfollow);
  

  void setPosition(double pos) {
    Arm.set(ControlMode.MotionMagic, pos);
    if (GetPosition() > UpperLimit) {
      Arm.configPeakOutputForward(0, 0);
    } else {
      Arm.configPeakOutputForward(1, 100);
    }
    if (GetPosition() < LowerLimit) {
      Arm.configPeakOutputReverse(0, 0);
    } else {
      Arm.configPeakOutputReverse(-1, 100);
    }
  }
  void ReadSensor() {
    Position = Arm.getSelectedSensorPosition();
  }

  double GetPosition() {   
    return Position;
  }
   
  void Config(){
    //Arm.configFactoryDefault();
    //Follow.configFactoryDefault();


    Arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,1, 100);
    Arm.setSensorPhase(true);
    Arm.setInverted(false);
    
    Arm.selectProfileSlot(2, 1);
    Arm.configNominalOutputForward(0, 100);
		Arm.configNominalOutputReverse(0, 100);
		Arm.configPeakOutputForward(1, 100);
    Arm.configPeakOutputReverse(-1, 100);
    
    Arm.configMotionCruiseVelocity(15000, 100);
    Arm.configMotionAcceleration(6000, 100);

    Follow.setInverted(true);
    Follow.follow(Arm);
  }

}
