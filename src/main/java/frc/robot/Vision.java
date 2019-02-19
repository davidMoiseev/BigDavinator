package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;


public class Vision extends Subsystem {

public double tv = 0; 
private  double tx = 0;
private double ty = 0;
private  double heading = 0;
private double targetHeading = 0;
private double hatchHeight = 28.875;
private double cargoHeight = 28.875;
private double limelightHeight = 10.75;
private double limelightAngle = 0.447;//-2.148;
private double a2 = 0.0;
private double distance = 0.0;

 @Override
  public void initDefaultCommand() {
     /* setDefaultCommand(new MySpecialCommand());
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("<tv>"). getDouble(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("<tx>"). getDouble(0);
    if (tv == 1) {
      boolean frc.robot.DriveTrain.turnComplete(tv);
 
    } else{
    }
    */
  }
  
  public void update()
  {
    getNetworkTables();
  }

  public void getNetworkTables() {
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv"). getDouble(0);
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx"). getDouble(0);
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty"). getDouble(0);
  }
public double getTV(){
  tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv"). getDouble(0);
  return tv;
}
public double getTX(){
  tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx"). getDouble(0);
  return tx;
}
public double getTY(){
  ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty"). getDouble(0);
  return ty;
}

public double getHeading(){   // could return targetHeading or heading using a boolean if already reached heading
  
  targetHeading = tx; 
    getNetworkTables();
    heading = tx; 
    return heading;
  } 
//}

public double findDistance(){
  // getNetworkTables();
  a2 = getTY(); 
 distance = (hatchHeight - limelightHeight) / Math.tan(Math.toRadians(limelightAngle + a2));
  // limelightAngle = Math.atan((h2-limelightHeight)/38) - a2; /*calculates angle of limelight crosshair relative to ground*/
  // return a1;
 double distanceInches = distance / 5.85;
 return distanceInches;
  
}


public double canSeeTarget() {
  tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv"). getDouble(0);
  return tv;
}

public void setPipeline(double pipeline){
  NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
}


}
