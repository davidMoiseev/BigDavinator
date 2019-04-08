package frc.robot.auto.sweetturn;

public class ThreePointInterpolation
{
	public static final int THREEPOINT_SIZE = 3;

	private Point2D[] map;

	public ThreePointInterpolation(Point2D[] mapping)
	{
		assert (mapping.length == THREEPOINT_SIZE);
		map = mapping;
	}

	public double GetMappedValue(double input)
	{
		double output;

		if (input > map[0].x && input < map[1].x)
		{
			output = interpolate(input, map[0].x, map[0].y, map[1].x, map[1].y);
		} else if (input >= map[1].x && input < map[2].x)
		{
			output = interpolate(input, map[1].x, map[1].y, map[2].x, map[2].y);
		} else if (input >= map[2].x)
		{
			output = map[2].y;
		} else if (input <= map[0].x)
		{
			output = map[0].y;
		} else
		{
			output = 0;
			System.out.println("Falling Through !!!!!!\n");
		}

		return output;

	}

	private double interpolate(double xInput, double x1, double y1, double x2, double y2)
	{
		if (x2 != x1)
			return (y2 - y1) / (x2 - x1) * (xInput - x1) + y1;
		else
		{
			System.out.println("Error Don't Divide By ZERO!!!!!!\n");
			return 0;
		}

	}
}