//Made by Nick S. and TJ M.
//Later Modified by Johnny B.

package frc.robot.manipulator;

import org.hotteam67.HotController;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotCommandProvider;
import frc.robot.constants.WiringIDs;

public class IntakePneumatics
{

	Solenoid intakeSingle = new Solenoid(WiringIDs.SOLENOID_INTAKE);

	boolean lastProvidedValue = false;
	boolean lastSetValue = false;

	public IntakePneumatics()
	{
	}

	public void Update(RobotCommandProvider command)
	{
		intakeSingle.set(command.SpearsClosed());
	}
	
	public boolean SpearsClosed()
	{
		return intakeSingle.get();
	}
}
