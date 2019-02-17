package frc.robot;

public interface IMotionMagicActuator{
    public void initialize();
    public void zeroSensors();
    public void displaySensorsValue();
    public void disable();

    public void setTarget(double target);
    public boolean reachedTarget();
    public int getError();
    public double getPosition();

}