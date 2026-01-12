package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;

//@TeleOp
public class Physics extends LinearOpMode {

    double angle = 50;
    double v0 = 50;

    double distance = 90; //diagonal length of the field: ~16.9705627485 ft
    final double height = 38.80; //38.80in

    /*public Physics(HardwareMap hardwareMap, Telemetry telemetry)
    {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }*/

    @Override
    public void runOpMode()
    {

        waitForStart();

        //v0 = findGoodVel(angle, distance);
        boolean justPressed = false;

        while (opModeIsActive())
        {
            if(gamepad1.dpad_up)
            {
                newAngle(angle + .001);
                justPressed = true;
                telemetry.addData("new angle", angle);
            }
            else if(gamepad1.dpad_down) {
                newAngle(angle - .001);
                justPressed = true;
                telemetry.addData("new angle", angle);
            }
            else if(gamepad1.dpad_right) {
                distance = restrictDist(distance + .001);
                justPressed = true;
                telemetry.addData("new dist", distance);
                telemetry.update();
            }
            else if(gamepad1.dpad_left) {
                distance = restrictDist(distance - .001);
                justPressed = true;
                telemetry.addData("new dist", distance);
                telemetry.update();
            }
            else if(justPressed){
                //v0 = findGoodVel(angle, distance);
                angle = findGoodAngle(v0, distance);
                justPressed = false;
            }

            if (isStopRequested()) return;
        }
    }

    public double findGoodVel(double inAngle, double inDistance)
    {
        if(inDistance == -Double.MAX_VALUE) return -100;
        double min = 1;
        double vel = 500;

        int count = 0;
        while(min > .25 || (min <= 0 && min != Double.MIN_VALUE)) {
            if(min == Double.MAX_VALUE) vel = restrictVel(100);
            if(min == Double.MIN_VALUE) vel = restrictVel(vel + 50);
            else if(min <= 0) vel = restrictVel(vel + 0.5 * min);
            else if(min > .25) vel = restrictVel(vel - 0.5 * min);
            min = getSmallestYDistance(inDistance, vel, inAngle);
            count++;
            if(count > 10000) return -100;
            telemetry.addLine("times ran " + count);
            telemetry.addData("velocity", vel);
            telemetry.update();
        }
        telemetry.addData("velocity", vel);
        telemetry.addData("angle", angle);
        telemetry.addData("min", min);
        return vel;
    }

    public double findGoodAngle(double inVel, double inDistance)
    {
        if(inDistance == -Double.MAX_VALUE) return -100;
        double min = 1;
        double ang = 45;

        double lastIterationAngle = 0;

        int count = 0;
        while(min > .25 || (min <= 0 && min != -Double.MAX_VALUE)) {
            if(min == Double.MAX_VALUE) ang = restrictAngle(70);
            if(min == -Double.MAX_VALUE) ang = restrictAngle(ang - 30);
            else if(min <= 0) ang = restrictAngle(ang - 0.005 * min);
            else if(min > .25) ang = restrictAngle(ang - 0.005 * min);
            if(lastIterationAngle-0.001 >= ang && lastIterationAngle+0.001 <= ang)
            {
                telemetry.addLine("same as last iteration");
                break;
            }
            lastIterationAngle = ang;
            min = getSmallestYDistance(inDistance, inVel, ang);
            count++;
            if(count > 1500) return ang;
            telemetry.addLine("21times ran " + count);
            telemetry.addData("angle", ang);
            telemetry.update();
        }
        telemetry.addData("angle", ang);
        telemetry.addData("velocity in in/s", inVel);
        telemetry.addData("dist in inches", distance);
        telemetry.addData("min", min);
        telemetry.update();
        return ang;
    }

    private void newAngle(double newAngle)
    {
        if(newAngle <= 0) angle = 0.0000001;
        else if(newAngle >= 90) angle = 89.9999999;
        else angle = newAngle;
    }
    private double restrictAngle(double newAngle)
    {
        if(newAngle <= 0) newAngle = 0.0001;
        else if(newAngle >= 90) newAngle = 89.9999;
        return newAngle;
    }
    private double restrictVel(double vel)
    {
        if(vel <= 0) vel = 0.000001;
        else if(vel >= 100000) vel = 1000000;
        return vel;
    }
    private double restrictDist(double dist)
    {
        if(dist < 0) dist = 0;
        else if(dist > 180) dist = 180;
        return dist;
    }

    public ArrayList<Point> getPath(double vel, double inAngle)
    {
        ArrayList<Point> points = new ArrayList<Point>();

        double x = 0;
        double y = 0;

        double velX = vel * Math.cos(Math.toRadians(inAngle));
        double velY = vel * Math.sin(Math.toRadians(inAngle));

        for(float t = 0; t < 3; t += 0.001)
        {
            x = velX * t;
            y = (velY * t) - (4.9 * Math.pow(t, 2));

            points.add(new Point(x,y));
        }
        return points;
    }
    public ArrayList<Point> getPath(double vel, double inAngle, int start, int end)
    {
        ArrayList<Point> points = new ArrayList<Point>();

        double x = 0;
        double y = 0;

        double velX = vel * Math.cos(Math.toRadians(inAngle));
        double velY = vel * Math.sin(Math.toRadians(inAngle));

        for(float t = start; t < end; t += 0.001)
        {
            x = velX * t;
            y = (velY * t) - (193.045 * Math.pow(t, 2)); //in/s/s

            points.add(new Point(x,y));
        }
        return points;
    }

    public double getSmallestYDistance(double inDistance, double vel, double inAngle)
    {
        boolean above = false;
        ArrayList<Point> nearWall = new ArrayList<Point>();
        ArrayList<Point> points = getPath(vel, inAngle);

        if(points.get(points.size() - 1).x - inDistance <= 0){
            telemetry.addLine("doesn't reach wall");
            return -Double.MAX_VALUE;
        }

        int start = -1;
        int end = -1;

        for(int i = 0; i < points.size(); i++)
        {
            Point p = points.get(i);
            if((-inDistance + p.x) > -0.25 && !above)
            {
                above = true;
                nearWall.add(p);
                start = i;
            }
            else if(above && (-inDistance + p.x) > 0.25)
            {
                nearWall.add(p);
                end = i;
                break;
            }
            if(above) nearWall.add(p);
        }

        points = getPath(vel, inAngle, start, end);

        double min = Double.MAX_VALUE;
        for(Point p : nearWall)
        {
            min = Math.min(p.y - height + 0.5, min);
        }
        return min;
    }
}