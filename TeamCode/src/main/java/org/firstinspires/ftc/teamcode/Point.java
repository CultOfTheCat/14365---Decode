package org.firstinspires.ftc.teamcode;

public class Point {
    public double x = 0;
    public double y = 0;
    public Point(double x, double y)
    {
        this.x = x;
        this.y = y;
    }
    @Override
    public String toString()
    {
        return "(" + (int)x + ", " + (int)y + ")";
    }
}

