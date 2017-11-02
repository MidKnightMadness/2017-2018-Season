package org.firstinspires.ftc.teamcode.MainBot.teleop.DriveAssembly.Tests.Gyro;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name = "Main Bot: Gyroo Display", group = "Mian Bot")
public class GyroDisplay extends LinearOpMode implements SensorEventListener   {

    private SensorManager sensorManager;
    private Sensor accelSensor;
    private Sensor magnetSensor;
    private Sensor gameSensor;
    private boolean mSRead = false;
    private boolean aSRead = false;
    private float[] mSensorReadings = new float[3];
    private float[] aSensorReadings = new float[3];
    private float[] orientation = new float[3];
    private float[] gSensorReadings = new float[3];
    private float[] rotationMatrix = new float[9];


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Status: Initialized and ready!");
        telemetry.update();

        sensorManager = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
        accelSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        magnetSensor = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        gameSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GEOMAGNETIC_ROTATION_VECTOR);


        sensorManager.registerListener(this, accelSensor,
                SensorManager.SENSOR_DELAY_NORMAL, SensorManager.SENSOR_DELAY_UI);
        sensorManager.registerListener(this, magnetSensor,
                SensorManager.SENSOR_DELAY_NORMAL, SensorManager.SENSOR_DELAY_UI);
        sensorManager.registerListener(this, gameSensor,
                SensorManager.SENSOR_DELAY_NORMAL, SensorManager.SENSOR_DELAY_UI);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Accel: " + aSensorReadings[0] + ", " + aSensorReadings[1] + ", " + aSensorReadings[2]);
            telemetry.addLine("Magnet: " + mSensorReadings[0] + ", " + mSensorReadings[1] + ", " + mSensorReadings[2]);
            telemetry.addLine("Orient: " + orientation[0] + ", " + orientation[1] + ", " + orientation[2]);
            telemetry.addLine("Game: " + gSensorReadings[0] + ", " + gSensorReadings[1] + ", " + gSensorReadings[2]);
            telemetry.update();
            idle();
        }
        // Do something useful

        sensorManager.unregisterListener(this);
    }

    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        if (sensorEvent.sensor == gameSensor) {
            gSensorReadings = sensorEvent.values;
        } else {
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
            }
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int value) {

    }
}