/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

/**
 * Add your docs here.
 */
public class ArmPigeon implements IPigeonWrapper
{

    private enum ArmPigeonState
    {
        UNINITALIZED, CALIBRATING, GATHERING_MEASUREMENTS, READY
    }

    private PigeonIMU pigeon;
    ArmPigeonState state;
    private double[] xyz_dps = new double[3];
    private double[] AverageAngle = new double[50];
    private int loops;
    private double angle;

    public ArmPigeon(int id)
    {
        pigeon = new PigeonIMU(id);
        state = ArmPigeonState.UNINITALIZED;
    }

    @Override
    public void CalibratePigeon()
    {
        short[] xyz_short = new short[3];
        double currentAngle = 1000;

        if (state == ArmPigeonState.UNINITALIZED)
        {
            pigeon.enterCalibrationMode(CalibrationMode.BootTareGyroAccel);
            state = ArmPigeonState.CALIBRATING;
        }
        if (state == ArmPigeonState.CALIBRATING)
        {
            if (pigeon.getState() == PigeonState.Ready)
            {
                state = ArmPigeonState.GATHERING_MEASUREMENTS;
            }
        }
        if (state == ArmPigeonState.GATHERING_MEASUREMENTS)
        {

            if (loops > 49)
            {
                loops = 49;
            }
            pigeon.getBiasedAccelerometer(xyz_short);
            // This converts the short value to a double
            for (int j = 0; j < xyz_short.length; j++)
            {
                xyz_dps[j] = (double) xyz_short[j];
                xyz_dps[j] = Math.round(xyz_dps[j]);
            }
            // convert the returned value into G's
            double x = (xyz_dps[0] / 16384);
            double y = (xyz_dps[1] / 16384);
            double z = (xyz_dps[2] / 16384);
            // Checks if Values Are Within The 1.1 G Tolerance value And Throws Out Bad
            // Values
            if ((x < -1.1 || x > 1.1) || (y < -1.1 || y > 1.1) || (z < -1.1 || z > 1.1))
            {
                loops = loops - 1;
            }
            else
            {
                // This converts the G values to radians then to degress
                currentAngle = Math.atan(z / ((Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)))));
                currentAngle = Math.toDegrees(currentAngle) - 90;
                // Allows The Rio Angle To Wrap Around
                if (z < 0)
                {
                    if (x > 0)
                    {
                        currentAngle = currentAngle + 90;
                    }
                    else
                    {
                        currentAngle = currentAngle - 90;
                    }
                }
                if (x < 0)
                {
                    AverageAngle[loops] = -currentAngle;
                }
                else
                {
                    AverageAngle[loops] = currentAngle;
                }
            }

            // This section does a rolling average and then returns the value
            if (loops >= 49)
            {
                double sum = 0;
                for (int i = 0; i < 50; i++)
                {
                    sum = AverageAngle[i] + sum;
                }
                angle = sum / 50;
                loops = 0;
                state = ArmPigeonState.READY;
            }
            else
            {
                loops++;
            }
        }
    }

    public double GetAngle()
    {
        return angle;
    }

    public void RestartCalibration()
    {
        state = ArmPigeonState.UNINITALIZED;
    }

    @Override
    public boolean PigeonReady()
    {
        return state == ArmPigeonState.READY;
    }
}
