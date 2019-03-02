//Made by Nick S. and TJ M.
//Later Modified by Johnny B.

package frc.robot;

import org.hotteam67.HotController;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.constants.IRobotCommandProvider;
import frc.robot.constants.WiringIDs;

public class IntakePneumatics
{

	Solenoid intakeSingle = new Solenoid(WiringIDs.SOLENOID_INTAKE);
	HotController joystick;

	public IntakePneumatics(HotController joystick)
	{
		this.joystick = joystick;
	}

	public void Update(IRobotCommandProvider command)
	{
		intakeSingle.set(command.IntakeSolenoid());
	}
}
