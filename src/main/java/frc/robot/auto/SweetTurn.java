/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.auto;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class SweetTurn
{
  private boolean complete;
  private double sweetTurnDirection;
  private double SWEET_TURN_ITERATE_MAX = 3;
  private double SWEET_TURN_MAX_EXIT_VELOCITY = 55.0;
  private double SWEET_TURN_PERCISE_TURN_PCT = 0.1;
  private double SWEET_TURN_RAMP_UP_RATE = 0.11;
  private double sweetturnstartdelay = 0;
  private double sweetTurnIterateCounter;
  private double SweetTurnMaxPct;
  private double sweetTurnRampDownRate;
  private double sweetTurnRate;
  private double sweetTurnRampDownStart;
  private double sweetTurnState = 0;
  private double sweetTurnTimer;
  private double sweetTurnTotalAngleTravel;
  private double sweetTurn_Precision = 4;
  private double sweetTurn_RampDown = 3;
  private double sweetTurn_Max = 2;
  private double sweetTurn_RampIn = 1;
  private double sweetTurn_reset = 0;
  private double sweetTurnOutput;

  public double sweetTurnMaxPct(double input)
  {
    double sweetTurnMaxPct = (0.3 + (1.05 * input * (-0.55 * (Math.pow(input, 2)))));
    SmartDashboard.putNumber("Max percent", sweetTurnMaxPct);
    return sweetTurnMaxPct;
  }

  public double sweetTurnRampDownStart(double input)
  {

    sweetTurnRampDownStart = (9.17130827 * Math.pow(1.008777513, input));
    SmartDashboard.putNumber("Ramp Down Start", sweetTurnRampDownStart);
    if (sweetTurnRampDownStart > 65)
    {
      sweetTurnRampDownStart = 120;
    }
    return (sweetTurnRampDownStart);
  }

  public double sweetTurnRampDownRate(double input)
  {

    sweetTurnRampDownRate = (1.536661436 * Math.pow(0.9494835049, input));
    SmartDashboard.putNumber("Ramp Down Rate", sweetTurnRampDownRate);
    return sweetTurnRampDownRate;
  }

  public void SweetTurnReset()
  {
    sweetTurnState = sweetTurn_reset;
  }

  public boolean IsTurnComplete()
  {
    return complete;
  }

  public double SweetTurnOutput(double target, double MinErrorToExit, double maxSpeed, double currentHeading, double currentSpeed)
  {
    complete = false;
    double absError = Math.abs(target - currentHeading);
    double maxPct;
    double rampDownStart;
    double remainingAngleAtStartRampDown = 0;
    SmartDashboard.putNumber("Error", absError);
    SmartDashboard.putNumber("State", sweetTurnState);
    SmartDashboard.putNumber("Yaw 2", currentHeading);

    if (sweetTurnIterateCounter > SWEET_TURN_ITERATE_MAX)
    {
      sweetTurnState = sweetTurn_reset;
      sweetTurnIterateCounter = 0;
      complete = true;
    }

    if (sweetTurnState != sweetTurn_reset && ((sweetTurnDirection == 1 && currentHeading > target)
        || (sweetTurnDirection == -1 && currentHeading < target)))
    {
      sweetTurnState = sweetTurn_reset;
      sweetTurnIterateCounter++;
    }

    if (sweetTurnState == sweetTurn_reset)
    {
      sweetTurnRate = 0;
      sweetTurnTimer = 0;
      sweetTurnState = sweetTurn_RampIn;
      sweetTurnTotalAngleTravel = absError;
      sweetTurnDirection = (target > currentHeading ? 1 : -1);
    }

    maxPct = this.sweetTurnMaxPct(sweetTurnTotalAngleTravel);

    if (maxPct > maxSpeed)
    {
      maxPct = maxSpeed;
    }

    SmartDashboard.putNumber("Max Speed", maxSpeed);

    rampDownStart = this.sweetTurnRampDownStart(Math.abs(currentSpeed));

    if (sweetTurnState == sweetTurn_RampIn)
    {
      sweetTurnRate += SWEET_TURN_RAMP_UP_RATE;

      if (absError <= MinErrorToExit && Math.abs(currentSpeed) <= SWEET_TURN_MAX_EXIT_VELOCITY)
      {
        sweetTurnRate = 0;
        sweetTurnTimer = 0;
        sweetTurnIterateCounter = 0;
        complete = true;
        sweetTurnState = sweetTurn_reset;
      }
      else if (absError <= rampDownStart)
      {
        sweetTurnState = sweetTurn_RampDown;
        remainingAngleAtStartRampDown = absError;
      }
      else if (sweetTurnRate >= maxPct)
      {
        sweetTurnState = sweetTurn_Max;
      }
    }

    if (sweetTurnState == sweetTurn_Max)
    {

      sweetTurnRate = maxPct;

      if (absError <= MinErrorToExit)
      {
        sweetTurnRate = 0;
        sweetTurnTimer = 0;
        sweetTurnIterateCounter = 0;
        complete = true;
        sweetTurnState = sweetTurn_reset;
      }
      else if (absError <= rampDownStart)
      {
        sweetTurnState = sweetTurn_RampDown;
        remainingAngleAtStartRampDown = absError;
      }
      else if (sweetTurnRate >= maxPct)
      {
        sweetTurnState = sweetTurn_Max;
      }
    }

    if (sweetTurnState == sweetTurn_RampDown)
    {
      sweetTurnRate -= this.sweetTurnRampDownRate(remainingAngleAtStartRampDown);

      if (absError <= MinErrorToExit && Math.abs(currentSpeed) <= SWEET_TURN_MAX_EXIT_VELOCITY)
      {
        sweetTurnRate = 0;
        sweetTurnTimer = 0;
        sweetTurnIterateCounter = 0;
        complete = true;
        sweetTurnState = sweetTurn_reset;
      }
      else if (sweetTurnRate <= SWEET_TURN_PERCISE_TURN_PCT)
      {
        sweetTurnState = sweetTurn_Precision;
      }
    }

    if (sweetTurnState == sweetTurn_Precision)
    {
      sweetTurnRate = SWEET_TURN_PERCISE_TURN_PCT;

      if (absError <= MinErrorToExit && Math.abs(currentSpeed) <= SWEET_TURN_MAX_EXIT_VELOCITY)
      {
        sweetTurnRate = 0;
        sweetTurnTimer = 0;
        sweetTurnIterateCounter = 0;
        complete = true;
        sweetTurnState = sweetTurn_reset;
      }
    }

    SmartDashboard.putNumber("Output", sweetTurnRate);
    SmartDashboard.putNumber("Direction", sweetTurnDirection);

    if (Math.abs(sweetTurnRate) >= maxSpeed)
    {
      sweetTurnOutput = maxSpeed;
    }
    else
    {
      sweetTurnOutput = sweetTurnRate;
    }
    SmartDashboard.putNumber("Output After", sweetTurnOutput);
    // driveTrain.arcadeDrive(-sweetTurnDirection * sweetTurnRate, 0);
    double turn = (-sweetTurnDirection * sweetTurnOutput);
    return turn;

  }

}
