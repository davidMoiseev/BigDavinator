/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FTC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionMotion {
    Vision vision = new Vision();

    public double h;
    public double speedH;
    public boolean isHDown;
    public double previousCanSeeTarget;
    public double prevprevCanSeeTarget;
    public static final double pGain = 0.04;
    public static final double iGain = 0.000008;
    public static final double dGain = 0.0;
    public static final double pGainBall = 0.04;
    public static final double iGainBall = 0.000008;
    public static final double dGainBall = 0.0;
    public double integral = 0;
    public double speed = 0;
    public double Lspeed;
    public double Rspeed;
    public double pAngle;
    public double iAngle;
    public double motorOutput;
    public int num;
    public double Houtput;
    public double Loutput;
    public double Routput;
    public double bouncePrevY = 0;

    public double turnVision() {
        vision.setPipeline(1);
        if (vision.canSeeTarget() == 0 || vision.getHeading() < 5.0 && vision.getHeading() > -5.0) {
            motorOutput = 0.0;
            return motorOutput;
        }
        /*
         * else if (vision.getHeading() == currentYaw) {
         * turnGyro(vision.getHeading(true));}
         */

        else if (vision.getHeading() > 5.0) {
            motorOutput = 0.4;
            return motorOutput;
        } else if (vision.getHeading() < -5.0) {
            motorOutput = -0.4;
            return motorOutput;
        }else{
            motorOutput = 0.0;
            return motorOutput;
        }

    }
    
    public double shuffleVision() {
        vision.setPipeline(1);
        if(vision.getHeading() < 4.0 && vision.getHeading() > -4.0 ) {   
            motorOutput = 0.0;
            return motorOutput;
        }
        /* else if (vision.getHeading() == currentYaw) {
        turnGyro(vision.getHeading(true));} */
    
        else if (vision.getHeading() > 4.0) {
            motorOutput = 0.25;
            return motorOutput;
    
        }
        else if (vision.getHeading() < -4.0) {
            motorOutput = -0.25;
            return motorOutput;
    
        }else{
            motorOutput = 0.0;
            return motorOutput;
        }
        }

    public double findProportional(double targetHeading) {
        double error = vision.getHeading() - targetHeading;
        return error;
    }

    public double findIntegral(double targetHeading) {
        integral = integral + this.findProportional(targetHeading);
        return integral;
    }

    public double shuffleVisionPID() {
        vision.setPipeline(1);
        pAngle = findProportional(0); // 27;
        iAngle = findIntegral(0); // 27;
        double Hspeed = (pAngle * pGain) + (iAngle * iGain);
        if (Hspeed > 1.0)
            Hspeed = 1.0;
        else if (Hspeed < -1.0) {
            Hspeed = -1.0;
        }
        //driveTrain.basicSideOutput(Hspeed);
        return Hspeed;
    }

     public double driveToVisionDistance() {
        if(vision.findDistance() < 20.0) {
            motorOutput = 0.0;
            return motorOutput;
        }else{
            motorOutput = 0.2;
            return motorOutput;
        }
    }

    public void followBallVis(){
        this.shuffleVisionPID();
        this.turnVision();
        this.driveToVisionDistance();
        double Lspeed = this.driveToVisionDistance() /*+ (this.turnVision())*/;
        double Rspeed = this.driveToVisionDistance() /*- (this.turnVision())*/;
        if(Lspeed > 1.0){
            Lspeed = 1.0;
        }else if(Lspeed < -1.0){
            Lspeed = -1.0;
        }
        if(Rspeed > 1.0){
            Rspeed = 1.0;
        }else if(Rspeed < -1.0){
            Rspeed = -1.0;
        }

            Loutput = Lspeed;
            Routput = Rspeed;
    }
    public boolean ballHasBeenFollowed(){
        if((Math.abs(vision.getHeading()) < 5.0) && (vision.findDistance()) < 20.0){
            return true;
        }else{
            return false;
        }
    }

    public double outputLBall(){
        this.followBallVis();
        double outputL = Loutput;
        return outputL;
    }

    public double outputRBall(){
        this.followBallVis();
        double outputR = Routput;
        return outputR;
    }

    public boolean detectBallBounce(){
        if(Math.abs(vision.getTY() - bouncePrevY) > 5.0){
            bouncePrevY = vision.getTY();
            return true;
        }else{
            bouncePrevY = vision.getTY();
            return false;
        }
    }

    public void writeDashBoardVis() {
        SmartDashboard.putNumber("heading", vision.getHeading());
        SmartDashboard.putNumber("Can Detect Target", vision.canSeeTarget());
        SmartDashboard.putNumber("distance", vision.findDistance());
        SmartDashboard.putBoolean("Is h down", isHDown);
        SmartDashboard.putBoolean("has ball been followed", this.ballHasBeenFollowed());
    }

};