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
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionMotion {
    Vision vision = new Vision();
    DriveTrain driveTrain = new DriveTrain();

    HotSticks hotDrive = new HotSticks(0);
    Solenoid solenoid = new Solenoid(0);
    public double h;
    public double speedH;
    public boolean isHDown;
    public double previousCanSeeTarget;
    public double prevprevCanSeeTarget;
    public static final double pGain = 0.04;
    public static final double iGain = 0.000008;
    public static final double dGain = 0.0;
    public double integral = 0;
    public double speed = 0;
    public double Lspeed;
    public double Rspeed;
    public double pAngle;
    public double iAngle;

    public void turnVision() {
        
        if(vision.canSeeTarget() == 0 || vision.getHeading() < 1.0 && vision.getHeading() > -1.0 ) {
            driveTrain.allOff();
        }
           /* else if (vision.getHeading() == currentYaw) {
            turnGyro(vision.getHeading(true));} */

           else if (vision.getHeading() > 1.0) {
               driveTrain.basicTurnOutput(0.4);
              }
            else if (vision.getHeading() < -1.0) {
              driveTrain.basicTurnOutput(-0.4);
            }; 
        
        }
        public void shuffleVision() {
        
            if(/*(vision.canSeeTarget() == 0 && previousCanSeeTarget == 0 && prevprevCanSeeTarget == 0) ||*/ vision.getHeading() < 4.0 && vision.getHeading() > -4.0 ) {
                driveTrain.dropH(false);
                driveTrain.basicSideOutput(0.0);
            }
               /* else if (vision.getHeading() == currentYaw) {
                turnGyro(vision.getHeading(true));} */
    
               else if (vision.getHeading() > 5.0) {
                driveTrain.dropH(true);    
                driveTrain.basicSideOutput(0.25);
               
                }
                else if (vision.getHeading() < -5.0) {
                    driveTrain.dropH(true);
                    driveTrain.basicSideOutput(-0.25);
                    
                }; 
            prevprevCanSeeTarget = previousCanSeeTarget;
            previousCanSeeTarget = vision.canSeeTarget();
            }    
          
        public double findProportional(double targetHeading){
            double error = vision.getHeading() - targetHeading;
            return error;
        }    
        public double findIntegral(double targetHeading) {
            integral = integral + this.findProportional(targetHeading);
            return integral;
        }   
        public void shuffleVisionPID(){
            driveTrain.dropH(true);

            pAngle = findProportional(0); // 27;
            iAngle = findIntegral(0); // 27;
            double Hspeed = (pAngle * pGain) + (iAngle * iGain);
            if (Hspeed > 1.0) 
                Hspeed = 1.0;
             else if (Hspeed < -1.0) {
                Hspeed = -1.0;
            }
            driveTrain.basicSideOutput(Hspeed);
        }
	
	public void writeDashBoardVis() {
        SmartDashboard.putNumber("heading", vision.getHeading());
        SmartDashboard.putNumber("H-Joystick", hotDrive.getStickRX());
        SmartDashboard.putNumber("Can Detect Target", vision.canSeeTarget());
        SmartDashboard.putBoolean("Is h down", isHDown);
    } 
    
}