package org.hotteam67;

import edu.wpi.first.wpilibj.Filesystem;

public class Path
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
