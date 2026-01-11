package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.getCurrentGameTagLibrary;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class BlueTeleop extends LinearOpMode
{
    /**
     * The variable to store our instance of the AprilTag processor.
     */

    /**
     * The variable to store our instance of the vision portal.
     */
    int manualPowerOffset = 0;
    @Override
    public void runOpMode() throws InterruptedException
    {
        getCurrentGameTagLibrary();
        Point center = new Point();
        ArrayList<Integer> colorList;
        AprilTagDetection aprilTagDetection = new AprilTagDetection(hardwareMap, telemetry, "blue");
        Intake intake = new Intake(hardwareMap, telemetry);
        intake.openBottomServo(false);
        intake.openTopServo(false);
        Lifting lift = new Lifting(hardwareMap, telemetry);

        Odometry odometry = new Odometry(hardwareMap, telemetry, "blue");

        float power = 1;
        boolean lastFrameDPad = false;

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        List<DcMotor> motors = List.of(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

        for(DcMotor motor : motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        aprilTagDetection.initAprilTag();

        if (isStopRequested()) return;


        int manualPosOffset = 0;

        boolean lastFrameLeft = false;
        boolean lastFrameRight = false;
        boolean lastFrameX = false;
        boolean lastFrameY = false;
        boolean lastFrameRightBumper2 = false;
        boolean lastFrameB2 = false;
        boolean lastFrameLeftTrigger2 = false;
        boolean lastFrameDpadUp2 = false;
        boolean lastFrameDpadDown2 = false;
        boolean lastFrameDpadRight2 = false;
        boolean lastFrameDpadLeft2 = false;

        boolean spinUpShootMotor = false;

        float veloToShoot = 0;

        while (opModeIsActive())
        {
            odometry.update();
            odometry.odometryTelemetry();
            center = aprilTagDetection.telemetryAprilTag();

            double y = gamepad1.right_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.left_stick_x;
            if(rx == 0) rx = gamepad2.right_stick_x/2;
            if(y == 0 && x == 0)
            {
                y = gamepad2.left_stick_y/2;
                x = -gamepad2.left_stick_x * 1.1/2;
            }

            double frontLeftPower = -(y + x + rx);
            double backLeftPower = -(y - x + rx);
            double frontRightPower = -(y - x - rx);
            double backRightPower = -(y + x - rx);

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            if(center.x < 920) telemetry.addLine("move left");
            if(center.x > 1120) telemetry.addLine("move right");

            //normal shooting
            if(gamepad2.right_trigger != 0)
            {
                if(aprilTagDetection.getDistance() < 0) telemetry.addLine("Camera cannot find AprilTag! \nplease try moving back");
                else
                {
                    veloToShoot = intake.getPolynomialPower((float)aprilTagDetection.getDistance()) + getManualOffset();
                }
            }
            if(veloToShoot != 0)
            {
                telemetry.addData("velocity", veloToShoot);
                if(intake.numShot == 4)
                {
                    intake.numShot = 0;
                    intake.shooting = false;
                    intake.loading = false;
                    veloToShoot = 0;
                }
                intake.shoot3(veloToShoot);
            }
            //shoot 1
            if(gamepad2.x)
            {
                intake.shoot(intake.getPolynomialPower((float)aprilTagDetection.getDistance()) + manualPowerOffset);
            }
            //toggle start up shoot motor
            if(gamepad2.left_trigger != 0 && !lastFrameLeftTrigger2)
            {
                spinUpShootMotor = !spinUpShootMotor;
                lastFrameLeftTrigger2 = true;
            }
            if(spinUpShootMotor) intake.shootMotor.setVelocity(-1000);
            //toggle top servo
            if(gamepad2.right_bumper && !lastFrameRightBumper2)
            {
                if(intake.servoTop.getPosition() == intake.topClosePos) intake.openTopServo(true);
                else intake.openTopServo(false);
                lastFrameRightBumper2 = true;
            }
            //turn to goal
            if(gamepad2.a && aprilTagDetection.getDistance() > 0)
            {
                aprilTagDetection.turnToCenterGoal(motors, .4f, 3, manualPosOffset);
            }

            if(gamepad2.dpad_up && !lastFrameDpadUp2)
            {
                manualPowerOffset += 25;
                lastFrameDpadUp2 = true;
            }
            if(gamepad2.dpad_down && !lastFrameDpadDown2)
            {
                manualPowerOffset -= 25;
                lastFrameDpadDown2 = true;
            }
            if(gamepad2.dpad_right && !lastFrameDpadRight2)
            {
                manualPosOffset += 10;
                lastFrameDpadRight2 = true;
            }
            if(gamepad2.dpad_left && !lastFrameDpadLeft2)
            {
                manualPosOffset -= 10;
                lastFrameDpadLeft2 = true;
            }
            //lifting?
            /*if(gamepad2.b && !lastFrameB2)
            {
                if(lift.getSlidesPos() == lift.slideStartPosition) lift.moveSlidesToPosition(10);
                else lift.moveSlidesToPosition(lift.slideStartPosition);
                lastFrameB2 = true;
            }*/



            if(aprilTagDetection.getDistance() < 0)
            {
                telemetry.addLine("|||||||| CANNOT FIND APRIL TAG!!! ||||||||");
            }
            else telemetry.addData("aprilTag distance", aprilTagDetection.getDistance());

            telemetry.addData("aprilTag X", center.x);
            telemetry.addData("-------> manual power offset", manualPowerOffset);
            telemetry.addData("-------> manual turn offset", manualPosOffset);

            lastFrameRight = gamepad1.dpad_right;
            lastFrameLeft = gamepad1.dpad_left;
            lastFrameDPad = gamepad1.dpad_down || gamepad1.dpad_up;
            lastFrameX = gamepad1.x;
            lastFrameY = gamepad1.y;
            if(!gamepad2.right_bumper) lastFrameRightBumper2 = false;
            if(!gamepad2.b) lastFrameB2 = false;
            if(gamepad2.left_trigger == 0) lastFrameLeftTrigger2 = false;
            if(!gamepad2.dpad_up) lastFrameDpadUp2 = false;
            if(!gamepad2.dpad_down)lastFrameDpadDown2 = false;
            if(!gamepad2.dpad_right) lastFrameDpadRight2 = false;
            if(!gamepad2.dpad_left) lastFrameDpadLeft2 = false;

            telemetry.update();
        }

    }
    float getManualOffset()
    {
        return manualPowerOffset;
    }
}