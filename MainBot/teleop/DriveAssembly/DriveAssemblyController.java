package org.firstinspires.ftc.teamcode.MainBot.teleop.DriveAssembly;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

public class DriveAssemblyController {
    
    private static double RATIO_WHEEL = 6300; //in encoder counts
    private static double RATIO_BOT = 1120;
    private static boolean MOTORS = true;
    private static double TURN_SPEED_RATIO = 1;

    private Telemetry telemetry;
    private DcMotor motorUp;
    private DcMotor motorDown;
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private double timeElapsed;
    private double timeThisRun;
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private double tempMotors[] = new double[4];
    private int oldMotorPos[] = new int[4];
    private int stopped = 0;
    private double theta = 0;
    private double translationDirection = 0;
    private double addedRotation = 0;

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;


        if (MOTORS) {
            motorUp = hardwareMap.dcMotor.get(CrossCommunicator.Drive.UP);
            motorDown = hardwareMap.dcMotor.get(CrossCommunicator.Drive.DOWN);
            motorLeft = hardwareMap.dcMotor.get(CrossCommunicator.Drive.LEFT);
            motorRight = hardwareMap.dcMotor.get(CrossCommunicator.Drive.RIGHT);

            motorUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        translationDirection = aTan(gamepad1.left_stick_x, -gamepad1.left_stick_y);

        //degrees per second to add to each motor speed
        addedRotation = gamepad1.right_stick_x * TURN_SPEED_RATIO;

        stopped = (gamepad1.left_stick_x == 0 && -gamepad1.left_stick_y == 0) ? 0 : 1;

        tempMotors[0] = stopped*Math.sin((theta + translationDirection)*(Math.PI/180d)) + addedRotation;
        tempMotors[1] = -stopped*Math.sin((theta + translationDirection)*(Math.PI/180d)) + addedRotation;
        tempMotors[2] = stopped*Math.cos((theta + translationDirection)*(Math.PI/180d)) + addedRotation;
        tempMotors[3] = -stopped*Math.cos((theta + translationDirection)*(Math.PI/180d)) + addedRotation;

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
        telemetry.addLine("Up Motor: " + tempMotors[3]);
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
            // dw/s * s = dw -> dw * db/dw = db
            timeThisRun = runtime.time() - timeElapsed;
            timeElapsed = runtime.time();

            theta += ((motorUp.getCurrentPosition() + motorDown.getCurrentPosition() + motorLeft.getCurrentPosition() + motorRight.getCurrentPosition() - oldMotorPos[0] - oldMotorPos[1] - oldMotorPos[2] - oldMotorPos[3])/4) * (RATIO_BOT / RATIO_WHEEL);

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


            telemetry.update();
    }

    public void stop() {

    }
}
