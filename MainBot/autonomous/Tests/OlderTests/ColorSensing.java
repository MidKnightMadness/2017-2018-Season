package org.firstinspires.ftc.teamcode.MainBot.autonomous.Tests.OlderTests;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Disabled
@Autonomous(name = "Color Sensing", group = "Main Bot")

public class ColorSensing extends LinearOpMode {

    ColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        colorSensor = hardwareMap.colorSensor.get("color");
        waitForStart();
        float hsvValues[] = {0F, 0F, 0F};
        while (opModeIsActive()) {
            //telemetry.addData("Alpha", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            //telemetry.addData("Color number ", colorSensor.argb());
            telemetry.addData("Red per blue", colorSensor.red()/colorSensor.blue());
            telemetry.addData("Blue per red", colorSensor.blue()/colorSensor.red());
            Color.RGBToHSV((int) (colorSensor.red() * 255),
                    (int) (colorSensor.green() * 255),
                    (int) (colorSensor.blue() * 255),
                    hsvValues);
            telemetry.addData("Hue ", hsvValues[0]);
            telemetry.addData("Saturation ", hsvValues[1]);
            telemetry.addData("Value ", hsvValues[2]);
            telemetry.update();
            idle();
        }
    }
}