package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.*;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;

public class Vision extends Subsystem
{

  public double tv = 0;
  private double tx = 0;
  private double ty = 0;
  private double ta = 0;
  private double heading = 0;
  private double targetHeight;
  private double rocketHeight = 38.000;
  private double normalHeight = 28.875;
  private double limelightHeight = 2.54;
  private double limelightAngle = 0.447;// -2.148;
  private double a2 = 0.0;
  private double distance = 0.0;
  private int brightness;
  private int exposure;

  UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();

  public void initDefaultCommand()
  {
    /*
     * setDefaultCommand(new MySpecialCommand());
     * NetworkTableInstance.getDefault().getTable("limelight").getEntry("<tv>").
     * getDouble(0);
     * NetworkTableInstance.getDefault().getTable("limelight").getEntry("<tx>").
     * getDouble(0); if (tv == 1) { boolean frc.robot.DriveTrain.turnComplete(tv);
     * 
     * } else{ }
     */
  }

  public void setUsbPipeline(int pipeline)
  {
    if (pipeline == 0)
    {
      brightness = 100000;
      exposure = 10000;
    }
    else if (pipeline == 1)
    {
      brightness = 2;
      exposure = 2;
    }
    else
    {
      brightness = 75;
      exposure = 35;
    }
  }

  public void usbCamInit()
  {
    new Thread(() ->
    {
      camera.setResolution(640, 480);

      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);

      Mat source = new Mat();
      Mat output = new Mat();

      setUsbPipeline(2);
      camera.setBrightness(brightness);
      camera.setExposureManual(exposure);

      while (!Thread.interrupted())
      {
        cvSink.grabFrame(source);
        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
        outputStream.putFrame(output);
      }
    }).start();

    CameraServer.getInstance().startAutomaticCapture();
  }

  public void usbCamUpdate(int pipeline)
  {
    setUsbPipeline(pipeline);
    camera.setBrightness(brightness);
    camera.setExposureManual(exposure);
  }

  public void getNetworkTables()
  {
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    tx =  NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    ty =  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  public double getTV()
  {
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    return tv;
  }

  public double getTX()
  {
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    return tx;
  }

  public double getTY()
  {
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    return ty;
  }

  public double getTA()
  {
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    return ta;
  }

  public double getHeading()
  { // could return targetHeading or heading using a boolean if already reached
    // heading
    getNetworkTables();
    heading = tx;
    return heading;
  }
  // }

  public double findDistance(int target)
  { // to the vision target, NOT THE BALL
    // getNetworkTables();

    if ((target == 1) || (target == 3))
    {
      targetHeight = rocketHeight;
    }
    else
    {
      targetHeight = normalHeight;
    }
    a2 = getTY();
    // distance = (targetHeight - limelightHeight) / Math.tan(Math.toRadians(limelightAngle + a2));
    limelightAngle = Math.atan((limelightHeight-limelightHeight)/38) - a2; /*calculates angle
    of limelight crosshair relative to ground*/
    return limelightAngle;
    //double distanceInches = distance / 5.85;
    // return distanceInches;

  }

  public double canSeeTarget()
  {
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    return 1;// tv;
  }

  public void setPipeline(double pipeline)
  {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
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
