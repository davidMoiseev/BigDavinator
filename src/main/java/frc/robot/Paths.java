package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;

public final class Paths
{
    public static class Path
    {
        public static final String Directory = Filesystem.getDeployDirectory() + "/output/";
        public static final String Ending = ".pf1.csv";

        public final String Left;
        public final String Right;

        public Path(String name)
        {
            Left = Directory + name + ".left" + Ending;
            Right = Directory + name + ".right" + Ending;
        }
    }

    public static final Path TestPath1 = new Path("testPath1");
    public static final Path TestPath2 = new Path("testPath2");
    public static final Path HatchPlaceRighttoHatchPickupRight = new Path("HatchPlaceRighttoHatchPickupRight");
}