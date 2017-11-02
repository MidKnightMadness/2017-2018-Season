package org.firstinspires.ftc.teamcode.MainBot.autonomous.Tests.Visual;

import android.graphics.Color;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * A simple test of a pair of color sensors
 */
@Disabled
@Autonomous(name="aJewel View 2", group ="Main Bot")

public class JewelView2 extends LinearOpMode
{
    AMSColorSensor leftColorSensor;
    AMSColorSensor rightColorSensor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        leftColorSensor  = (AMSColorSensor)hardwareMap.colorSensor.get("leftColorSensor");
        rightColorSensor = (AMSColorSensor)hardwareMap.colorSensor.get("rightColorSensor");

        AMSColorSensor.Parameters params = leftColorSensor.getParameters();
        // possibly change some (notably gain and / or integration time), then
        // leftColorSensor.initialize(params);

        params = rightColorSensor.getParameters();
        // possibly change some (notably gain and / or integration time), then
        // rightColorSensor.initialize(params);

        waitForStart();

        while (opModeIsActive())
        {
            int left = leftColorSensor.argb();
            int right = rightColorSensor.argb();
            telemetry.addData("left", String.format("a=%d r=%d g=%d b=%d", Color.alpha(left), Color.red(left), Color.green(left), Color.blue(left)));
            telemetry.addData("right", String.format("a=%d r=%d g=%d b=%d", Color.alpha(right), Color.red(right), Color.green(right), Color.blue(right)));
            this.updateTelemetry(telemetry);

            Thread.sleep(500);
        }

    }
}