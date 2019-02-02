package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;


public class Vision extends Subsystem {

public double tv = 0; 
private  double tx = 0;
private  double heading = 0;
private double targetHeading = 0;

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

  }

public double getHeading(){   // could return targetHeading or heading using a boolean if already reached heading
  
  targetHeading = tx; 
  /* if (sensor == true) {
    return targetHeading;
  }
  else {
    */
    getNetworkTables();
    heading = tx; 
    return heading;
  } 
//}


public double canSeeTarget() {
  tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv"). getDouble(0);
  return tv;
}
}