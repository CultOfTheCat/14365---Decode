package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;

//@TeleOp
public class Intake extends LinearOpMode
{
    Telemetry telemetry;
    int shootingPosition = 1;

    private NormalizedColorSensor colorSensor;

     DcMotorEx shootMotor;
    Servo servoBottom, servoTop;
    final float bottomClosePos = 0.45f;
    final float bottomOpenPos = 0.55f;
    final float topClosePos = 0.7f;
    final float topOpenPos = 0.9f;
    //dist, power
    private Map<Integer, Integer> powers = new HashMap<>();


    //The value used to determine if there is a ball between the servos
    final double ballMaxDistance = 10;
    private DistanceSensor ballDistanceSensor;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry)
    {
        this.telemetry = telemetry;

        shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");

        servoBottom = hardwareMap.get(Servo.class, "bottomIntakeServo");
        servoTop = hardwareMap.get(Servo.class, "topIntakeServo");

        ballDistanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        powers.put(94,1850);
        powers.put(93, 1850);
        powers.put(92,1850);
        powers.put(91, 1800);
        powers.put(90, 1800);
        powers.put(89, 1750);
        powers.put(88, 1750);
        powers.put(87, 1725);
        powers.put(86, 1725);
        powers.put(85,1725);
        powers.put(84, 1700);
        powers.put(83, 1700);
        powers.put(82, 1700);
        powers.put(81, 1700);
        powers.put(80, 1700);
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }

    //R^2 = 0.967
    public float getPower(float dist)
    {
        return (float)(-3*Math.pow(10,-8)*Math.pow(dist,4) + 7*Math.pow(10,-6)*Math.pow(dist,3) - 0.0005*Math.pow(dist,2) + 0.0193*dist + 0.4762);
    }

    public float getPolynomialPower(float dist)
    {
        float x = dist; //12393.71 - 915.885*x + 27.84441*Math.pow(x,2) - 0.3649013*Math.pow(x,3) + 0.001754586*Math.pow(x,4)
        if(x < 60) return (float)(12333.71 - 915.885*x + 27.84441*Math.pow(x,2) - 0.3649013*Math.pow(x,3) + 0.001754586*Math.pow(x,4));
        else
        {
            if(powers.get((int)Math.ceil(dist)) == null || powers.get((int)Math.floor(dist)) == null)
            {
                if(Math.ceil(dist) < 80) return 1675;
                else return 1900;
            }
            else
            {
                float power = (powers.get((int)Math.ceil(dist)) + powers.get((int)Math.floor(dist))) / 2;
                return power;
            }
//            float power = (float)(-101683.3 + 3852.621*x - 47.82096*Math.pow(x,2) + 0.19780809*Math.pow(x,3));
//            return Math.max(1730, power);
        }
    }

    //R^2 = 0.9593
    public float getPowLinear(float dist)
    {
        return (float)(0.0027*dist + 0.6567);
    }

    public void shoot(float power)
    {
        if (!isBallInChamber()) return;

        //spin shootMotor
        shootMotor.setVelocity(-power);
        //wait seconds

        sleepTime(4000);
        loadNextBall();
    }
    Timer timer = new Timer();
    public void shoot3Timed(float power)
    {
        shootMotor.setVelocity(-power);
        sleepTime(5000);
        loadNextBall();
        sleepTime(1300);
        loadNextBall();
        sleepTime(1300);
        loadNextBall();
        openTopServo(true);

        shootMotor.setVelocity(0);
    }
    boolean shooting = false;
    boolean loading = false;
    int numShot = 1;
    public void shoot3(float power)
    {
        shootMotor.setVelocity(-power);
        if(!shooting)
        {
            shooting = true;
        }
        if(!isBallInChamber())
        {
            shooting = false;
            shootMotor.setVelocity(0);

            loading = true;
        }
        if(shooting && isAtVelo(power))
        {
            loading = true;
        }
        if(loading)
        {
            loadNextBall();
        }
        if(numShot == 4) return;
    }

    long time = 0;
    long startTime = 0;
    void loadNextBall()
    {
        if(startTime == 0) startTime = System.currentTimeMillis();
        time = System.currentTimeMillis() - startTime;

        if(time < 500);
        else if(time < 800)
        {
            openBottomServo(true);
        }
        else if(time < 1300)
        {
            openBottomServo(false);
        }
        else if(time < 1700)
        {
            openTopServo(true);
        }
        else if(time < 2300)
        {
            openTopServo(false);
        }
        else
        {
            startTime = 0;
            time = 0;
            loading = false;
            numShot++;
        }
/*
        //wait less than second
        sleepTime(500);
        //close servoBottom

        sleepTime(75);


        //wait <second
        sleepTime(500);
        //close servoTop

        loading = false;*/
    }
    void dropBall()
    {

    }
    private void waitForVelo(float power)
    {
        long startTime = System.currentTimeMillis();
        while(Math.abs(shootMotor.getVelocity()) < Math.min(1780,power) && System.currentTimeMillis() - startTime < 3000)
        {

        }
        sleepTime(500);
    }
    private boolean isAtVelo(float power)
    {
        return Math.abs(shootMotor.getVelocity()) > Math.min(1780, Math.abs(power));
    }

    private void sleepTime(long ms)
    {
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - startTime < ms)
        {

        }
    }

    public void openBottomServo(boolean open)
    {
        servoBottom.setPosition(open ? bottomOpenPos : bottomClosePos);
    }
    public void openTopServo(boolean open)
    {
        servoTop.setPosition(open ? topOpenPos : topClosePos);
    }

    boolean isBallInChamber() { return ballDistanceSensor.getDistance(DistanceUnit.CM) < ballMaxDistance; }
}