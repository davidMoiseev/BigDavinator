package frc.robot.constants;

public class AutonCommandProvider implements IRobotCommandProvider
{

    @Override
    public void Update() {

    }

    @Override
    public frc.robot.constants.ManipulatorSetPoint ManipulatorSetPoint() {
        return ManipulatorSetPoint.carry_front;
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

}