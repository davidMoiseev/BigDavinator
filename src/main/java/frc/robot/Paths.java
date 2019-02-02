package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;

public final class Paths
{
    public static final String Directory = Filesystem.getDeployDirectory() + "/output/";
    public static final String testLeft = Directory + "testPath1.left.pf1.csv";
    public static final String testRight = Directory + "testPath1.right.pf1.csv";
}