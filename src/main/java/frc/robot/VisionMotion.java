/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FTC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import org.graalvm.compiler.nodeinfo.StructuralInput.Memory;

//import static org.junit.Assert.assertArrayEquals;

//import com.sun.org.apache.xpath.internal.axes.SelfIteratorNoPredicate;
//import com.sun.tools.javac.jvm.Target;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionMotion {
    Vision vision = new Vision();

    public double h;
    public double speedH;
    public boolean isHDown;
    public double previousCanSeeTarget;
    public double prevprevCanSeeTarget;
    public static final double pGainH = 0.04;
    public static final double iGainH = 0.000008;
    public static final double dGainH = 0.0;
    public static final double pGainTurn = 0.04;
    public static final double iGainTurn = 0.0;//0.0000000008;
    public static final double dGainTurn = 0.0;
    public double integral = 0;
    public double speed = 0;
    public double Lspeed;
    public double Rspeed;
    public double pAngle;
    public double iAngle;
    public double p;
    public double i;
    public double motorOutput;
    public int num;
    public double Houtput;
    public double Loutput;
    public double Routput;
    public double gyroLoutput;
    public double gyroRoutput;
    public double gyroHoutput;
    public double bouncePrevY = 0;
    public double earlierCanSeeTarget = 0.0;
    public double prevHeadingVis = 0.0;
    public int gyroState = 0;
    public double targetAngle;
    public double targetVisDistance = 17.0; //inches, 30.125 + distance btwn robot perimeter and camera, aka constant C
    public double distanceDiagonal;
    public double distanceHorizontal;
    public double angle2;
    public double angle1;
    public double vy;
    public double vx;
    public double vyh;
    public double vylr;
    public double vxh;
    public double vxlr;
    public double distance = 0;

    public double findProportional(double targetHeading) {
        double error = vision.getHeading() - targetHeading;
        return error;
    }

    public double findIntegral(double targetHeading) {
        integral = integral + this.findProportional(targetHeading);
        return integral;
    }

    public double turnVision() {
        p = findProportional(0); 
        i = findIntegral(0); 
        if (vision.canSeeTarget() == 0 || vision.getHeading() < 5.0 && vision.getHeading() > -5.0) {
            motorOutput = 0.0;
            return motorOutput;
        }
         else if (Math.abs(vision.getHeading()) > 5.0) {
             motorOutput = (p * pGainTurn) + (i * iGainTurn);
            if (motorOutput > 0.4){
                motorOutput = 0.4;
            }else if(motorOutput < -0.4){
                motorOutput = -0.4;
            }
            return motorOutput;
        }else{
            motorOutput = 0.0;
            return motorOutput;
        }

    }
    
    public boolean definitelySeesTarget(){
        double maxHeadingChange = 1000;
        if ( (earlierCanSeeTarget == vision.getTV()) && (Math.abs(prevHeadingVis - vision.getTV()) < maxHeadingChange)){
            earlierCanSeeTarget = vision.getTV();
            prevHeadingVis = vision.getHeading();
            return true;
        }else{
            earlierCanSeeTarget = vision.getTV();
            prevHeadingVis = vision.getHeading();
            return false;
        }

    }

    public double shuffleVisionPID() {
        pAngle = findProportional(0); // 27;
        iAngle = findIntegral(0); // 27;
        double Hspeed = (pAngle * pGainH) + (iAngle * iGainH);
        if (Hspeed > 0.5)
            Hspeed = 0.5;
        else if (Hspeed < -0.5) {
            Hspeed = -0.5;
        }
        //driveTrain.basicSideOutput(Hspeed);
        if(this.definitelySeesTarget() == true){
            return Hspeed;
        }else{
            return 0;
        }
    }

     public double driveToVisionDistance() {
        if(vision.findDistance() < 20.0) {
            motorOutput = 0.0;
            return motorOutput;
        }else{
            motorOutput = 0.4;
            return motorOutput;
        }
    }

    public void setPipeline(double pipeline){
        vision.setPipeline(pipeline);
    }

    public void targetLineUp(){
        this.shuffleVisionPID();
        this.turnVision();
        this.driveToVisionDistance();
        double Lspeed = this.driveToVisionDistance() + (this.turnVision());
        double Rspeed = this.driveToVisionDistance() - (this.turnVision());
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
        if(this.definitelySeesTarget() == true){
            Loutput = Lspeed;
            Routput = Rspeed;
        }
    }

    public boolean targetFound(){
        if((Math.abs(vision.getHeading()) < 5.0) && (vision.findDistance()) < 20.0){
            return true;

        }else{
            return false;
        }
    }

    public double outputL(){
        this.targetLineUp();
        double outputL = Loutput;
        return outputL;
    }

    public double outputR(){
         this.targetLineUp();
        double outputR = Routput;
        return outputR;
    }

    public void getTargetAngle(double yaw){
        targetAngle = yaw + Math.toRadians(vision.getHeading());
    }

    public boolean setGyroLineUpVars(){
        distance = vision.findDistance();
        distanceHorizontal = vision.findDistance() * Math.tan(targetAngle);
        angle2 = Math.atan((vision.findDistance() - targetVisDistance) / distanceHorizontal);
        angle1 = ((Math.PI / 2) - targetAngle) - angle2;
        return true;
    }
    public void gyroTargetLineUp(double yaw, double vt){ //vt is the motor output of the resultant
               // vy = Math.tan(yaw * vision.getHeading()) * ;//Math.sin(yaw) * vt;
                //vx = //Math.cos(yaw)* vt; 
                // vxh = vx * Math.cos(yaw);
                // vxlr = vx * Math.sin(yaw);
                // vyh = vy * Math.sin(yaw); 
                // vylr = vy * Math.cos(yaw);
                //gyroLoutput = vy;//(vylr + vxlr);
               // gyroRoutput = vy;//(vylr + vxlr);
                //gyroHoutput = vx;//(vyh + vxh);
                gyroLoutput = (distance * vt) / (distance + Math.abs(distanceHorizontal));
                gyroRoutput = (distance * vt) / (distance + Math.abs(distanceHorizontal));
                gyroHoutput = (distanceHorizontal * vt) / (distanceHorizontal + distance);
                SmartDashboard.putNumber("distanceHorizontal", distanceHorizontal);
                SmartDashboard.putNumber("atan(targetAngle)", Math.atan(targetAngle));
                SmartDashboard.putNumber("targetVisDistance", targetVisDistance);
                SmartDashboard.putNumber("angle2", angle2);
                SmartDashboard.putNumber("angle1", angle1);
                SmartDashboard.putNumber("vy", vy);
                SmartDashboard.putNumber("vx", vx);
                SmartDashboard.putNumber("vt", vt);
                // SmartDashboard.putNumber("vxh", vxh);
                // SmartDashboard.putNumber("vxlr", vxlr);
                // SmartDashboard.putNumber("vyh", vyh);
                // SmartDashboard.putNumber("vylr", vylr);
             }

    // public void gyroTargetLineUp(double yaw, double vt){ //vt is the motor output of the resultant
    //     double distance = vision.findDistance();
    //     distanceHorizontal = 40 * Math.tan(Math.PI / 4);
    //     angle2 = Math.atan((40 - targetVisDistance) / distanceHorizontal);
    //     angle1 = ((Math.PI / 2) - (Math.PI / 4)) - angle2;
    //             vy = Math.sin(Math.PI / 4) * vt;  
    //             vx = Math.cos(Math.PI / 4)* vt; 
    //             // vxh = vx * Math.cos(yaw);
    //             // vxlr = vx * Math.sin(yaw);
    //             // vyh = vy * Math.sin(yaw);
    //             // vylr = vy * Math.cos(yaw);
    //             // gyroLoutput = vy;//(vylr + vxlr);
    //             // gyroRoutput = vy;//(vylr + vxlr);
    //             // gyroHoutput = vx;//(vyh + vxh);
    //             gyroLoutput = (40 * vt) / (40 + distanceHorizontal);

    //             gyroRoutput = (40 * vt) / (40 + distanceHorizontal);
    //             gyroHoutput = (distanceHorizontal * vt) / (distanceHorizontal + 40);
    //             SmartDashboard.putNumber("distanceHorizontal", distanceHorizontal);
    //             SmartDashboard.putNumber("atan(targetAngle)", Math.atan(targetAngle));
    //             SmartDashboard.putNumber("targetVisDistance", targetVisDistance);
    //             SmartDashboard.putNumber("angle2", angle2);
    //             SmartDashboard.putNumber("angle1", angle1);
    //             SmartDashboard.putNumber("vy", vy);
    //             SmartDashboard.putNumber("vx", vx);
    //             SmartDashboard.putNumber("vt", vt);
    //             // SmartDashboard.putNumber("vxh", vxh);
    //             // SmartDashboard.putNumber("vxlr", vxlr);
    //             // SmartDashboard.putNumber("vyh", vyh);
    //             // SmartDashboard.putNumber("vylr", vylr);
          

    //     }

    public boolean reachedTarget() {
        //distanceDiagonal = Math.sqrt((distance * distance) + (distanceHorizontal * distanceHorizontal));
        if(vision.findDistance() <= targetVisDistance){
            return true;
        }else{
            return false;
        }
    }

    
    public double outputGyroL(double yaw, double maxMotorOutput){
       // this.gyroTargetLineUp(yaw, maxMotorOutput);
        double outputL = gyroLoutput;
        return outputL;
    }

    public double outputGyroR(double yaw, double maxMotorOutput){
       // this.gyroTargetLineUp(yaw, maxMotorOutput);
        double outputR = gyroRoutput;
        return outputR;
    }

    public double outputGyroH(double yaw, double maxMotorOutput){
       // this.gyroTargetLineUp(yaw, maxMotorOutput);
        double outputH = gyroHoutput;
        return outputH;
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
        SmartDashboard.putBoolean("has ball been followed", this.targetFound());
        SmartDashboard.putNumber("targetAngle", targetAngle);
    }

};