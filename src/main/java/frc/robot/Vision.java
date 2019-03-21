package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.*;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;

public class Vision
{

  private final String networkTable;
  public double tv = 0;
  private double tx = 0;
  private double ty = 0;
  private double ta = 0;
  private double rocketHeight = 38.000;
  private final double hatchHeight = 28.875;
  private final double limelightHeight;
  private final double limelightX;

  public Vision(String name)
  {
    networkTable = name;
    limelightHeight = 31;
    limelightX = 5;
  }

  public Vision(String name, double X, double height)
  {
    networkTable = name;
    limelightX = X;
    limelightHeight = height;
  }

  public void getNetworkTables()
  {
    tv = NetworkTableInstance.getDefault().getTable(networkTable).getEntry("tv").getDouble(0);
    tx = NetworkTableInstance.getDefault().getTable(networkTable).getEntry("tx").getDouble(0);
    ty = NetworkTableInstance.getDefault().getTable(networkTable).getEntry("ty").getDouble(0);
  }

  public double getTV()
  {
    tv = NetworkTableInstance.getDefault().getTable(networkTable).getEntry("tv").getDouble(0);
    return tv;
  }

  public double getHorizontal()
  {
    double hz = NetworkTableInstance.getDefault().getTable(networkTable).getEntry("ty").getDouble(0);
    return hz;
  }

  public double getVertical()
  {
    double vt = NetworkTableInstance.getDefault().getTable(networkTable).getEntry("tx").getDouble(0);
    return vt;
  }

  public double getSkew()
  {
    double skew = NetworkTableInstance.getDefault().getTable(networkTable).getEntry("ts").getDouble(0);
    return skew;
  }

  public double getTA()
  {
    ta = NetworkTableInstance.getDefault().getTable(networkTable).getEntry("ta").getDouble(0);
    return ta;
  }

  public double getHeading()
  { // could return targetHeading or heading using a boolean if already reached
    // heading
    getNetworkTables();
    return ty;
  }
  // }

  public double findDistance()
  { // to the vision target, NOT THE BALL
    double angle = getVertical();
    double heightDif = limelightHeight - hatchHeight;
    double dist = heightDif / Math.tan(Math.toRadians(angle));
    return dist;
  }

  public double findX(double dist, double horizontal)
  {
    return limelightX - Math.sin(horizontal) * dist;
  }

  public double angleFromCenter()
  {
    double dist = findDistance();
    double horizontal = getHorizontal();
    return Math.atan(limelightX + (Math.sin(horizontal) * dist) / (dist * Math.cos(horizontal)));
  }

  public double canSeeTarget()
  {
    tv = NetworkTableInstance.getDefault().getTable(networkTable).getEntry("tv").getDouble(0);
    return 1;// tv;
  }

  public void setPipeline(double pipeline)
  {
    NetworkTableInstance.getDefault().getTable(networkTable).getEntry("pipeline").setNumber(pipeline);
  }

  public double findDistance(int target)
  { // to the vision target, NOT THE BALL
    return 0;
  }

}

/*
 * ------A Shrine to the Limelight------ ..........{-=-<( 55 )>-=-}.......... *
 * * * -=<[{ We Are }]>=- * * * * ____The Holy Number Plus Twelve____ ______And
 * Hallowed Be Thy Name_____ ______----------_______ __-- --__ - - --__ __--
 * ------_________------
 * 
 * -----May We Reach LimeLightenment-----
 */