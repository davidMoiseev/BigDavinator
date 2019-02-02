package frc.robot;

public interface IMotionMagicActuator{
    public void initialize();
    public void getError();
    public void zeroSensors();
    public void displaySensorsValue();
    public void disable();

    public void setTarget(double target);

}