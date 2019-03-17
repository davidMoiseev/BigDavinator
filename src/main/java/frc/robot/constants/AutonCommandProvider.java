package frc.robot.constants;

public class AutonCommandProvider implements IRobotCommandProvider
{

    @Override
    public void Update() {

    }

    @Override
    public IManipulatorSetPoint ManipulatorSetPoint() {
        return null;
    }

    @Override
    public boolean ManipulatorScore() {
        return false;
    }

    @Override
    public double LeftDrive() {
        return 0;
    }

    @Override
    public double RightDrive() {
        return 0;
    }

    @Override
    public boolean steeringAssistActivated()
    {
        return false;
    }

    @Override
    public double LeftDriveSteeringAssist() {
        return 0;
    }

    @Override
    public double RightDriveSteeringAssist() {
        return 0;
    }

    @Override
    public double HDrive() {
        return 0;
    }

    @Override
    public boolean IntakeSolenoid() {
        return false;
    }

    @Override
    public boolean IntakeOut() {
        return false;
    }

    @Override
    public boolean IntakeIn() {
        return false;
    }

    @Override
    public boolean ClimberDeploy()
    {
        return false;
    }

    @Override
    public boolean HatchPickup()
    {
        return false;
    }

    @Override
    public boolean ARMREZERO() {
        return false;
    }

    @Override
    public void SetIntakeSolenoid(boolean isTrue)
    {
        return;
    }
}