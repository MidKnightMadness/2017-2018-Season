package org.firstinspires.ftc.teamcode.MainBot.autonomous.Tests;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@Autonomous(name = "New Auto Test", group = "Test")
public class NewAutoTest extends LinearOpMode {

    private LynxI2cColorRangeSensor c;
    @Override
    public void runOpMode() throws InterruptedException {

        //Init here
        telemetry.addLine("Status: Initialized and ready!");
        telemetry.update();
        c = (LynxI2cColorRangeSensor) hardwareMap.get("color");
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Distance: ", c.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }

        // Do something useful

    }
}