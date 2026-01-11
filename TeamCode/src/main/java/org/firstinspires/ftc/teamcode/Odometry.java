package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Odometry extends LinearOpMode {
    DcMotorEx leftDeadwheel;
    DcMotorEx rightDeadwheel;
    DcMotorEx perpDeadwheel;

    final static double deadwheelRadius = 1.6; //cm
    final static double ticksPerRev = -2003.5;
    final static double deadwheelDistance = 35.25; //cm; distance between deadwheels //13.875in

    final static double perpDistance = 17.78; //cm

    static double angle0; //leftDeadwheel.getCurrentPosition() - rightDeadwheel.getCurrentPosition() / deadwheelDistance;

    String color;

    private static final Point blueGoalPosition = new Point(26, 22);
    private static final Point redGoalPosition = new Point(340, 22);

    public Odometry(HardwareMap hardwareMap, Telemetry telemetry, String color)
    {
        this.leftDeadwheel = hardwareMap.get(DcMotorEx.class, "leftSlide"); //left tick/rev = -2002.6 (forward)
        this.rightDeadwheel = hardwareMap.get(DcMotorEx.class, "rightSlide"); //right tick/rev = -2004.4 (forward)
        this.perpDeadwheel = hardwareMap.get(DcMotorEx.class, "perpendicularOdo");

        leftDeadwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDeadwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpDeadwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.color = color;
    }

    static double currentAngle; //degrees

    static double cmLeft;
    static double cmRight;
    static double cmPerp;

    static double x0 = 0;
    static double y0 = 0;

    double deltaCenterX;
    double deltaCenterY;
    double currentX; //cm
    double currentY; //cm

    public void update()
    {
        //field 365.75cm x 365.75cm
        cmLeft = (double)leftDeadwheel.getCurrentPosition() / ticksPerRev  * 2 * Math.PI * deadwheelRadius;
        cmRight = (double)rightDeadwheel.getCurrentPosition() / ticksPerRev * 2 * Math.PI * deadwheelRadius;
        cmPerp = (double)perpDeadwheel.getCurrentPosition() / ticksPerRev * 2 * Math.PI * deadwheelRadius;

        currentAngle = angle0 + ((cmLeft - cmRight) / deadwheelDistance);
        currentAngle %= 2 * Math.PI;

        deltaCenterX = ((cmLeft + cmRight) / 2);
        deltaCenterY = (cmPerp - (perpDistance * currentAngle));

        currentX = x0 + (deltaCenterX * Math.cos(Math.toRadians(currentAngle))) - (deltaCenterY * Math.sin(Math.toRadians(currentAngle)));
        currentY = y0 + (deltaCenterX * Math.sin(Math.toRadians(currentAngle))) + (deltaCenterY * Math.cos(Math.toRadians(currentAngle)));
    }
    public void odometryTelemetry()
    {
        //telemetry.addData("\noriginal angle", angle0);
        telemetry.addData("delta angle", currentAngle);

        telemetry.addData("\nx", currentX);
        telemetry.addData("y", currentY);
        telemetry.addData("deltaCenterX", deltaCenterX);
        telemetry.addData("deltaCenterY", deltaCenterY);
        telemetry.addData("currentAngle", Math.toDegrees(currentAngle));
    }
    public double getAngle(AngleUnit unit)
    {
        if(unit == AngleUnit.DEGREES) return Math.toDegrees(currentAngle);
        if(unit == AngleUnit.RADIANS) return currentAngle;
        else return -404;
    }
    public double getAngle()
    {
        return getAngle(AngleUnit.DEGREES);
    }
    public Point getPosition()
    {
        return new Point(currentX, currentY);
    }
    public double getX()
    {
        return currentX;
    }
    public double getY()
    {
        return currentY;
    }

    public double getDistToGoal()
    {
        if(color.equals("blue"))
        {
            double a = Math.abs(getX() - blueGoalPosition.x);
            double b = Math.abs(getY() - blueGoalPosition.y);
            return Math.sqrt(Math.pow(a,2) + Math.pow(b,2));
        }
        else if(color.equals("red"))
        {
            double a = Math.abs(getX() - redGoalPosition.x);
            double b = Math.abs(getY() - redGoalPosition.y);
            return Math.sqrt(Math.pow(a,2) + Math.pow(b,2));
        }
        return -404;
    }
    public double getAngleToGoal()
    {
        if(color.equals("blue"))
        {
            double a = Math.abs(getX() - blueGoalPosition.x); //adjacent
            double b = Math.abs(getY() - blueGoalPosition.y); //opposite

            return Math.atan(b/a);
        }
        if(color.equals("red"))
        {
            double a = Math.abs(getX() - redGoalPosition.x); //adjacent
            double b = Math.abs(getY() - redGoalPosition.y); //opposite

            return Math.atan(b/a);
        }
        return -404;
    }

    public void runOpMode()
    {

    }
}