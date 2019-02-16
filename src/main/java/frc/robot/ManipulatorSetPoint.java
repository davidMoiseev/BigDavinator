/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.constants.ManipulatorSetPoints;

public class ManipulatorSetPoint {
    private double armAngle;
    private double wristAngle;
    private double elevatorHeight;
    private double overallHeight;
    private double angle;
    public ManipulatorSetPoint(double angle, double height) {
        this.overallHeight = height;
        this.angle = angle;
    }


    public double getArmAngle() {
        return armAngle;
    }
    
    public void setArmAngle(double armAngle) {
        this.armAngle = armAngle;
    }

    public double getElevatorHeight() {
        return elevatorHeight;
    }

    public void setElevatorHeight(double elevatorHeight) {
        this.elevatorHeight = elevatorHeight;
    }

    public double getWristAngle() {
        return wristAngle;
    }

    public void setWristAngle(double wristAngle) {
        this.wristAngle = wristAngle;
    }
}