package frc.robot.constants;

public interface IRobotCommandProvider
{
    public void Update();

    public ManipulatorSetPoint ManipulatorSetPoint();

    public boolean ManipulatorScore();

    public double LeftDrive();

    public double RightDrive();

    public double HDrive();

    public boolean IntakeSolenoid();

    public boolean IntakeOut();

    public boolean IntakeIn();
}