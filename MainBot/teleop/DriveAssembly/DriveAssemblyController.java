package org.firstinspires.ftc.teamcode.MainBot.teleop.DriveAssembly;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

public class DriveAssemblyController implements SensorEventListener{
    
    private static double RATIO_WHEEL = 17.5; //in encoder counts
    private static double RATIO_BOT = 1;
    private static boolean MOTORS = false;
    private static double TURN_SPEED_RATIO = 1;
    private static boolean THETA_BY_PHONE = true;

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

    private SensorManager sensorManager;
    private Sensor accelSensor;
    private Sensor magnetSensor;
    private boolean mSRead = false;
    private boolean aSRead = false;
    private float[] mSensorReadings = new float[3];
    private float[] aSensorReadings = new float[3];
    private float[] orientation = new float[3];
    private float[] rotationMatrix = new float[9];
    private double startPos = 0;

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

            motorUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            oldMotorPos = new int[]{motorUp.getCurrentPosition(), motorDown.getCurrentPosition(), motorLeft.getCurrentPosition(), motorRight.getCurrentPosition()};
        }
        if (THETA_BY_PHONE) {
            sensorManager = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
            accelSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
            magnetSensor = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);


            sensorManager.registerListener(this, accelSensor,
                    SensorManager.SENSOR_DELAY_NORMAL, SensorManager.SENSOR_DELAY_UI);
            sensorManager.registerListener(this, magnetSensor,
                    SensorManager.SENSOR_DELAY_NORMAL, SensorManager.SENSOR_DELAY_UI);
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
        if (!THETA_BY_PHONE) {
            // dw/s * s = dw -> dw * db/dw = db
            timeThisRun = runtime.time() - timeElapsed;
            timeElapsed = runtime.time();

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
        } else {
            telemetry.addLine("Orient: " + orientation[0] + ", " + orientation[1] + ", " + orientation[2]);
        }

            telemetry.update();
    }

    public void stop() {
        sensorManager.unregisterListener(this);
    }

    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        if (sensorEvent.sensor == accelSensor) {
            aSensorReadings = sensorEvent.values;
            aSRead = true;
        } else if (sensorEvent.sensor == magnetSensor) {
            mSensorReadings = sensorEvent.values;
            mSRead = true;
        }

        if (aSRead && mSRead) {
            SensorManager.getRotationMatrix(rotationMatrix, null, aSensorReadings, mSensorReadings);
            SensorManager.getOrientation(rotationMatrix, orientation);
            orientation[0] *= 180d/Math.PI;
            orientation[1] *= 180d/Math.PI;
            orientation[2] *= 180d/Math.PI;
            theta = orientation[0] - startPos;
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int value) {}
}
