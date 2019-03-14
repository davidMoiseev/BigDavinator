package frc.robot.constants;

public interface IRobotCommandProvider
{
    public void Update();

    public IManipulatorSetPoint ManipulatorSetPoint();

    public boolean ManipulatorScore();

    public double LeftDrive();

    public double RightDrive();

    public double LeftDriveSteeringAssist();

    public double RightDriveSteeringAssist();

    public double HDrive();

    public boolean IntakeSolenoid();

    public boolean IntakeOut();

    public boolean IntakeIn();

    public boolean ClimberDeploy();

    public boolean HatchPickup();

    public boolean ARMREZERO();

    public boolean steeringAssistActivated();
}