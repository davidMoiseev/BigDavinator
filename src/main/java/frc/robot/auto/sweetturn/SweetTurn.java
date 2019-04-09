package frc.robot.auto.sweetturn;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.hotteam67.HotLogger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SweetTurn
{
	private enum sweetTurnSt
	{
		sweetTurn_reset, sweetTurn_RampIn, sweetTurn_Max, sweetTurn_RampDown, sweetTurn_Precision
	};

	private ThreePointInterpolation sweetTurnMaxPct;
	private ThreePointInterpolation sweetTurnRampDownStart;
	private ThreePointInterpolation sweetTurnRampDownRate;

	private sweetTurnSt sweetTurnState = sweetTurnSt.sweetTurn_reset;
	private double sweetTurnTimer = 0;
	private double sweetTurnRate;
	private double sweetTurnTotalAngleTravel;
	private double sweetTurnDirection; // should only ever be -1 or 1
	private double sweetTurnIterateCounter = 0;
	private boolean complete = false;

	public SweetTurn()
	{
		sweetTurnMaxPct = new ThreePointInterpolation(Constants.SweetTurnMaxPct);
		sweetTurnRampDownStart = new ThreePointInterpolation(Constants.SweetTurnRampDownStartOffSet);
		sweetTurnRampDownRate = new ThreePointInterpolation(Constants.SweetTurnRampDownRate);
	}

	// SWEET PROFILES
	public void SweetTurnRestart()
	{
		sweetTurnRate = 0;
		sweetTurnState = sweetTurnSt.sweetTurn_reset;
		sweetTurnTotalAngleTravel = 0;
		sweetTurnDirection = 0;
		sweetTurnTimer = 0;
		sweetTurnIterateCounter = 0;
		complete = false;
	}

	public double SweetTurnOutput(double target, double MinErrorToExit, double maxSpeed, double currentHeading,
			double currentTurnSpeed)
	{
		double absError = Math.abs(target - currentHeading);
		double maxPct;
		double rampDownStart;
		double remainingAngleAtStartRampDown = 360;

		if (sweetTurnIterateCounter > Constants.SWEET_TURN_ITERATE_MAX)
		{
			sweetTurnState = sweetTurnSt.sweetTurn_reset;
			sweetTurnIterateCounter = 0;
			complete = true;
		}

		if (sweetTurnState != sweetTurnSt.sweetTurn_reset && ((sweetTurnDirection == 1 && currentHeading > target)
				|| (sweetTurnDirection == -1 && currentHeading < target)))
		{
			sweetTurnState = sweetTurnSt.sweetTurn_reset;
			sweetTurnIterateCounter++;
		}

		if (sweetTurnState == sweetTurnSt.sweetTurn_reset)
		{
			sweetTurnRate = 0;
			sweetTurnTimer = 0;
			sweetTurnState = sweetTurnSt.sweetTurn_RampIn;
			sweetTurnTotalAngleTravel = absError;
			sweetTurnDirection = target > currentHeading ? 1 : -1;
		}

		maxPct = sweetTurnMaxPct.GetMappedValue(sweetTurnTotalAngleTravel);
		if (maxPct > maxSpeed)
		{
			maxPct = maxSpeed;
		}

		rampDownStart = sweetTurnRampDownStart.GetMappedValue(Math.abs(currentTurnSpeed));

		HotLogger.Log("rampDownStart", rampDownStart);
		if (sweetTurnState == sweetTurnSt.sweetTurn_RampIn)
		{
			sweetTurnRate += Constants.SWEET_TURN_RAMP_UP_RATE;

			if (absError <= MinErrorToExit && Math.abs(currentTurnSpeed) <= Constants.SWEET_TURN_MAX_EXIT_VELOCITY)
			{
				sweetTurnRate = 0;
				sweetTurnTimer = 0;
				sweetTurnIterateCounter = 0;
				complete = true;
				sweetTurnState = sweetTurnSt.sweetTurn_reset;
			}
			else if (absError <= rampDownStart)
			{
				sweetTurnState = sweetTurnSt.sweetTurn_RampDown;
				remainingAngleAtStartRampDown = absError;
			}
			else if (sweetTurnRate >= maxPct)
			{
				sweetTurnState = sweetTurnSt.sweetTurn_Max;
			}
		}

		if (sweetTurnState == sweetTurnSt.sweetTurn_Max)
		{

			sweetTurnRate = maxPct;

			if (absError <= MinErrorToExit)
			{
				sweetTurnRate = 0;
				sweetTurnTimer = 0;
				sweetTurnIterateCounter = 0;
				complete = true;
				sweetTurnState = sweetTurnSt.sweetTurn_reset;
			}
			else if (absError <= rampDownStart)
			{
				sweetTurnState = sweetTurnSt.sweetTurn_RampDown;
				remainingAngleAtStartRampDown = absError;
			}
			else if (sweetTurnRate >= maxPct)
			{
				sweetTurnState = sweetTurnSt.sweetTurn_Max;
			}
		}

		if (sweetTurnState == sweetTurnSt.sweetTurn_RampDown)
		{
			sweetTurnRate -= sweetTurnRampDownRate.GetMappedValue(remainingAngleAtStartRampDown);
			HotLogger.Log("sweetTurnRampDown", sweetTurnRampDownRate.GetMappedValue(remainingAngleAtStartRampDown));

			if (absError <= MinErrorToExit && Math.abs(currentTurnSpeed) <= Constants.SWEET_TURN_MAX_EXIT_VELOCITY)
			{
				sweetTurnRate = 0;
				sweetTurnTimer = 0;
				sweetTurnIterateCounter = 0;
				complete = true;
				sweetTurnState = sweetTurnSt.sweetTurn_reset;
			}
			else if (sweetTurnRate <= Constants.SWEET_TURN_PERCISE_TURN_PCT)
			{
				sweetTurnState = sweetTurnSt.sweetTurn_Precision;
			}
		}

		if (sweetTurnState == sweetTurnSt.sweetTurn_Precision)
		{
			sweetTurnRate = Constants.SWEET_TURN_PERCISE_TURN_PCT;

			if (absError <= MinErrorToExit && Math.abs(currentTurnSpeed) <= Constants.SWEET_TURN_MAX_EXIT_VELOCITY)
			{
				sweetTurnRate = 0;
				sweetTurnTimer = 0;
				sweetTurnIterateCounter = 0;
				complete = true;
				sweetTurnState = sweetTurnSt.sweetTurn_reset;
			}
		}

		SmartDashboard.putString("AAA Sweet Turn State", sweetTurnState.name());
		SmartDashboard.putNumber("AAA Sweet Heading", currentHeading);
		SmartDashboard.putNumber("AAA Target", target);

		return -sweetTurnDirection * sweetTurnRate;
	}

	public boolean TurnComplete()
	{
		return complete;
	}

	public static final List<String> LoggerTags = new ArrayList<>(Arrays.asList("rampDownStart", "sweetTurnRampDown"));
}