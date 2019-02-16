//Made By Nick S. and TJ M.

package frc.robot;

import org.hotteam67.HotController;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.constants.WiringIDs;
 


public class Pneumatics {

	Solenoid intakeSingle = new Solenoid(WiringIDs.SOLENOID_INTAKE);
	HotController joystick;
	
	public Pneumatics(HotController joystick) {
		this.joystick = joystick;
	}

	public void Update()
	{
		if(joystick.getButtonX()) 
		{
	
            intakeSingle.set(true);
            
		}
		if(joystick.getButtonY()) 
		{

			intakeSingle.set(false);

		}
	}
}
	