package frc.robot.auto;

public class Turn
{
    private double target = 0;
    private double lastOutput = 0;

    public static final double P = .005;
    public static final double RAMP_UP = .005;
    public static final double RAMP_DOWN = .005;
    public static final double MAX_OUTPUT = .6;
    public static final double MIN_OUTPUT = .055;
    public static final double ALLOWABLE_ERROR = 15;

    public void Reset()
    {
        target = 0;
        lastOutput = 0;
    }

    public double TurnToAngle(double angle, double currentHeading)
    {
        this.target = angle;

        double out = P * (angle - currentHeading);

        // Ramp up cap
        if (((out < 0 && lastOutput > out) || (out > 0 && lastOutput < out)) && (Math.abs(lastOutput - out) > RAMP_UP))
        {
            out = lastOutput + (RAMP_UP * Math.signum(out));
        }
        // Ramp down cap
        else if (((out < 0 && lastOutput < out) || (out > 0 && lastOutput > out))
                && (Math.abs(lastOutput - out) > RAMP_DOWN))
        {
            out = lastOutput - (RAMP_DOWN * Math.signum(out));
        }

        if (Math.abs(out) > MAX_OUTPUT)
            out = MAX_OUTPUT * Math.signum(out);
        if (Math.abs(out) < MIN_OUTPUT)
            out = MIN_OUTPUT * Math.signum(out);

        return out;
    }

    public boolean TurnOnTarget(double currentHeading)
    {
        return (Math.abs(target - currentHeading) < ALLOWABLE_ERROR);
    }
}