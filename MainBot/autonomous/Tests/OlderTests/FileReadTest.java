package org.firstinspires.ftc.teamcode.MainBot.autonomous.Tests.OlderTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;

@Disabled
@Autonomous(name = "File Read Test", group = "File")
public class FileReadTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //Init here

        telemetry.addLine("Status: Initialized and ready!");
        telemetry.update();

        waitForStart();

        try {
            FileInputStream f = new FileInputStream("/storage/self/primary/LastTeamColor.txt");
            if (f.available() >= 1) {
                int test = f.read();
                telemetry.addData("Data: ", test);
                telemetry.update();
            } else {
                telemetry.addData("Available: ", f.available());
                telemetry.update();
            }
        } catch (Exception e) {
            telemetry.addData("Error: ", e);
            telemetry.update();
        }

        while (opModeIsActive()) {
            idle();
        }
    }
}