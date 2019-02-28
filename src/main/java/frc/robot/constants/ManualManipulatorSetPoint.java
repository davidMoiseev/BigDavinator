package frc.robot.constants;

public class ManualManipulatorSetPoint implements IManipulatorSetPoint
{

    private final double armAngle;
    private final double wristAngle;
    private final double elevatorHeight;
    private final double frontFlipper;
    private final double backFlipper;
    public ManualManipulatorSetPoint(double armAngle, double wristAngle, double elevatorHeight, double frontFlipper, double backFlipper)
    {
        this.armAngle = armAngle;
        this.wristAngle = wristAngle;
        this.elevatorHeight = elevatorHeight;
        this.frontFlipper = frontFlipper;
        this.backFlipper = backFlipper;
    }

    @Override
    public double armAngle()
    {
        return armAngle;
    }

    @Override
    public double wristAngle()
    {
        return wristAngle;
    }

    @Override
    public double elevatorHeight()
    {
        return elevatorHeight;
    }

    @Override
    public double frontFlipper()
    {
        return frontFlipper;
    }

    @Override
    public double backFlipper()
    {
        return backFlipper;
    }
    
}