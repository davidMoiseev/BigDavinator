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
    public double currentYaw;
    public double referenceAngle;
    private int target;


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
        if(vision.findDistance(this.selectTarget()) < 20.0) {
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

    public void sendAngle(double yaw){
        if((currentYaw < 2 * Math.PI) && (currentYaw > -2 * Math.PI)){
            currentYaw = yaw;
        }
        while (currentYaw > 2 * Math.PI){
            currentYaw = currentYaw -2 * Math.PI;
        }
        while (currentYaw < -2 * Math.PI){
            currentYaw = currentYaw + 2 * Math.PI;
        }
    }

    public boolean getHigh(){ //true = high, false = low
        if (vision.getTY() > 6) {
            return true;
        }
        else if (vision.getTY() < 3) {
            return false;
        }
        else if (vision.getTY() > 3 && vision.getTY() < 6) {
            if(vision.getTA() < 5) {
                return true;
            }
            else {
                return false;
            }
        }else{
            return false;
        }
    }
    

    public int selectTarget(){
        if((currentYaw < (Math.PI / 8)) && (currentYaw > (Math.PI / -8))){ //if angle is zero
            referenceAngle = 0;
            target = 0; //front cargo ship
            
        }
        else if((currentYaw > (3 * Math.PI / 8)) && (currentYaw < (5 * Math.PI / -8))){ //if angle is pi/2
            referenceAngle = Math.PI / 2;
            if(getHigh() == true){ //if it is high
                target = 1; //right rocket center
            }else if(getHigh() == false){ //if it is low
                target = 2; //left side cargo
            }   
        }
        else if((currentYaw < (-3 * Math.PI / 8)) && (currentYaw > (-5 * Math.PI / -8))){ //if angle is -pi/2
            referenceAngle = Math.PI / -2;
            if(getHigh() == true){ //if it is high
                target = 3; //left rocket center
            }else if( getHigh() == false){ //if it is low
                target = 4; //right side cargo
            } 
        }
        else if((currentYaw > (Math.PI / 8)) && (currentYaw < (3 * Math.PI / 8))){ //if angle is pi/4
            referenceAngle = Math.PI / 4;
            target = 5; //right rocket near
            
        }
        else if((currentYaw < (-Math.PI / 8)) && (currentYaw > (-3 * Math.PI / 8))){ //if angle is -pi/4
            referenceAngle = Math.PI / -4;
            target = 6; //left rocket near
            
        }
        else if((currentYaw > (5 * Math.PI / 8)) && (currentYaw < (7 * Math.PI / 8))){ //if angle is 3pi/4
            referenceAngle = 3 * Math.PI / 4;
            target = 7; //right rocket far 
        }
        else if((currentYaw < (-5 * Math.PI / 8)) && (currentYaw > (-7 * Math.PI / 8))){ //if angle is -3pi/4
            referenceAngle = -3 * Math.PI / 4;
            target = 8; //left rocket far 
        }
        else if((currentYaw > (Math.PI / -8)) && (currentYaw < (Math.PI / 8))){ //if angle is zero
            referenceAngle = (Math.PI);
            target = 0; //front cargo ship
        }
        return target;
    }

    public double getReferenceAngle(){
        return referenceAngle;
    }

    public void targetLineUp(){ //faster but sloppier than gyroTargetLineUp
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

    public double outputL(){ //for targetLineUp
        this.targetLineUp();
        double outputL = Loutput;
        return outputL;
    }

    public double outputR(){ //for targetLineUp
         this.targetLineUp();
        double outputR = Routput;
        return outputR;
    }

    public void getTargetAngle(){
        targetAngle = referenceAngle + Math.toRadians(vision.getHeading());
    }

    public boolean setGyroLineUpVars(){
        distance = vision.findDistance(this.selectTarget());
        distanceHorizontal = distance * Math.tan(targetAngle);
        angle2 = Math.atan((distance - targetVisDistance) / distanceHorizontal);
        angle1 = ((Math.PI / 2) - targetAngle) - angle2;
        SmartDashboard.putNumber("distance", distance);
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
                SmartDashboard.putNumber("vt", vt);
             }

    public boolean targetReached(double target) {
        //distanceDiagonal = Math.sqrt((distance * distance) + (distanceHorizontal * distanceHorizontal));
        if(vision.findDistance(this.selectTarget()) <= target){
            return true;
        }else{
            return false;
        }
    }

    
    public double outputGyroL(double yaw, double maxMotorOutput){
        double outputL = gyroLoutput;
        return outputL;
    }

    public double outputGyroR(double yaw, double maxMotorOutput){
        double outputR = gyroRoutput;
        return outputR;
    }

    public double outputGyroH(double yaw, double maxMotorOutput){
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
        SmartDashboard.putBoolean("Is h down", isHDown);
        SmartDashboard.putNumber("targetAngle", targetAngle);
        SmartDashboard.putNumber("distanceHorizontal", distanceHorizontal);
        SmartDashboard.putNumber("targetVisDistance", targetVisDistance);
        SmartDashboard.putNumber("angle2", angle2);
        SmartDashboard.putNumber("angle1", angle1);
        SmartDashboard.putNumber("vy", vy);
        SmartDashboard.putNumber("vx", vx);
        SmartDashboard.putNumber("atan(targetAngle)", Math.atan(targetAngle));
    }

};