package org.firstinspires.ftc.teamcode;

public class Pose extends Point {
    //in radians pls
    public double angle = 0;
    
    public Pose(double x, double y)
    {
        super(x, y);
    }
    public Pose(double x, double y, double angle)
    {
        super(x, y);
        this.angle = angle;
    }
    
    public double getAngleDegrees()
    {
        return Math.toDegrees(angle);
    }
    
    @Override
    public String toString()
    {
        return "(" + (int)x + ", " + (int)y + ") angle: " + getAngleDegrees() + " degrees";
    }
}
