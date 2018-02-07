package org.firstinspires.ftc.teamcode.MainBot.teleop.DriveAssembly.Tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

public class DriveAssemblyOldController {

    private static double RATIO_WHEEL = 6300; //in encoder counts
    private static double RATIO_BOT = 1120;
    private static boolean MOTORS = true;
    private static double TURN_SPEED_RATIO = 1;
    private static boolean THETA_BY_GYRO = true;
    private boolean isUsingTheta = true;
    private boolean bPressed = false;

    private Telemetry telemetry;
    private DcMotor motorUp;
    private DcMotor motorDown;
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private double timeElapsed;
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private double tempMotors[] = new double[4];
    private int oldMotorPos[] = new int[4];
    private int stopped = 0;
    private double theta = 0;
    private double translationDirection = 0;
    private double addedRotation = 0;
    private BNO055IMU imu;
    private double startPos = 0;

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        if (MOTORS) {
            motorUp = hardwareMap.dcMotor.get(CrossCommunicator.Drive.FRONT_LEFT);
            motorDown = hardwareMap.dcMotor.get(CrossCommunicator.Drive.BACK_RIGHT);
            motorLeft = hardwareMap.dcMotor.get(CrossCommunicator.Drive.BACK_LEFT);
            motorRight = hardwareMap.dcMotor.get(CrossCommunicator.Drive.FRONT_RIGHT);

            motorUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            oldMotorPos = new int[]{motorUp.getCurrentPosition(), motorDown.getCurrentPosition(), motorLeft.getCurrentPosition(), motorRight.getCurrentPosition()};
        }
        telemetry.addLine("Status: Initialized and Ready!");
        telemetry.update();
    }

    private double aTan(double x, double y) {
        double a = 0;
        if (y == 0) {
            if (x < 0) {
                a = -90;
            } else if (x > 0) {
                a = 90;
            } else if (x == 0) {
                a = 0;
            }
        } else if (x == 0) {
            a = 0;
        } else {
            a = (Math.atan(x/y)*(180/Math.PI));
        }

        if (y < 0) {
            if (x < 0) {
                a -= 180;
            } else {
                a += 180;
            }
        }
        return a;
    }

    public void start() {
        timeElapsed = runtime.time();
        startPos = theta;
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad1.a) {
            startPos = AngleUnit.normalizeDegrees(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
            theta = 0;
        }
        if (gamepad1.b && !bPressed) {
            isUsingTheta = !isUsingTheta;
            if (isUsingTheta) {
                startPos = AngleUnit.normalizeDegrees(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
                theta = startPos;
            }
            bPressed = true;
        } else if (!gamepad1.b){
            bPressed = false;
        }



        translationDirection = aTan(gamepad1.left_stick_x, -gamepad1.left_stick_y);

        //degrees per second to add to each motor speed
        addedRotation = gamepad1.right_stick_x * TURN_SPEED_RATIO;

        stopped = (gamepad1.left_stick_x == 0 && -gamepad1.left_stick_y == 0) ? 0 : 1;

        tempMotors[0] = stopped*Math.cos((theta + translationDirection)*(Math.PI/180d)) + addedRotation;
        tempMotors[1] = -stopped*Math.cos((theta + translationDirection)*(Math.PI/180d)) + addedRotation;
        tempMotors[2] = stopped*Math.sin((theta + translationDirection)*(Math.PI/180d)) + addedRotation;
        tempMotors[3] = -stopped*Math.sin((theta + translationDirection)*(Math.PI/180d)) + addedRotation;


        double scale = Math.max(Math.max(Math.abs(tempMotors[0]), Math.abs(tempMotors[1])), Math.max(Math.abs(tempMotors[2]), Math.abs(tempMotors[3])));
        if (scale == 0) {
            scale = 0;
        } else {
            scale = 0.8/scale;
        }

        tempMotors[0] *= scale;
        tempMotors[1] *= scale;
        tempMotors[2] *= scale;
        tempMotors[3] *= scale;
        //telemetry.addLine("Gamepad Left Stick X: " + gamepad1.left_stick_x);
        //telemetry.addLine("Gamepad Left Stick Y: " + -gamepad1.left_stick_y);
        //telemetry.addLine("Gamepad Right Stick X: " + gamepad1.right_stick_x);
        telemetry.addLine("Theta: " + theta);
        //telemetry.addLine("Translation Direction: " + translationDirection);
        //telemetry.addLine("Added Rotation: " + addedRotation);
        //telemetry.addLine("Time Elapsed: " + timeElapsed);
        //telemetry.addLine("Scale: " + scale);
        telemetry.addLine("Up Motor: " + tempMotors[0]);
        telemetry.addLine("Down Motor: " + tempMotors[1]);
        telemetry.addLine("Left Motor: " + tempMotors[2]);
        telemetry.addLine("Right Motor: " + tempMotors[3]);

        //telemetry.update();

        if (MOTORS) {
            motorUp.setPower(tempMotors[0]);
            motorDown.setPower(tempMotors[1]);
            motorLeft.setPower(tempMotors[2]);
            motorRight.setPower(tempMotors[3]);
        }


        /* ************SET THETA************ */
        if (!THETA_BY_GYRO && isUsingTheta) {
            // dw/s * s = dw -> dw * db/dw = db

            //encoder ticks of motor * (encoder ticks of bot / encoder ticks of wheel) * (degrees of bot / encoder ticks of bot)
            theta += ((motorUp.getCurrentPosition() + motorDown.getCurrentPosition() + motorLeft.getCurrentPosition() + motorRight.getCurrentPosition() - oldMotorPos[0] - oldMotorPos[1] - oldMotorPos[2] - oldMotorPos[3]) / 4d) * (RATIO_BOT / RATIO_WHEEL);

            telemetry.addLine("Up Motor: " + (motorUp.getCurrentPosition() - oldMotorPos[0]));
            telemetry.addLine("Down Motor: " + (motorDown.getCurrentPosition() - oldMotorPos[1]));
            telemetry.addLine("Left Motor: " + (motorLeft.getCurrentPosition() - oldMotorPos[2]));
            telemetry.addLine("Right Motor: " + (motorRight.getCurrentPosition() - oldMotorPos[3]));

            oldMotorPos = new int[]{motorUp.getCurrentPosition(), motorDown.getCurrentPosition(), motorLeft.getCurrentPosition(), motorRight.getCurrentPosition()};

            /*telemetry.addData("Gamepad Right Stick X: ", gamepad1.right_stick_x);
            telemetry.addData("Time Elapsed: ", timeThisRun);
            telemetry.addData("Rot * Scale * Time Elapsed: ", addedRotation * scale * timeThisRun);
            telemetry.addData("Ratio: ", RATIO_BOT / RATIO_WHEEL);
            telemetry.addData("All Together: ", addedRotation * scale * timeThisRun * (RATIO_BOT / RATIO_WHEEL));
            telemetry.addData("Theta: ", theta);*/
        } else if (isUsingTheta){
            theta = AngleUnit.normalizeDegrees(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle) - startPos;
        }
            telemetry.update();
    }

    public void stop() {
    }
}
