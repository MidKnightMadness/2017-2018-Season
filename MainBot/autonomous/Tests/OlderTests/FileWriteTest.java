package org.firstinspires.ftc.teamcode.MainBot.autonomous.Tests.OlderTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;
import java.io.FileOutputStream;

@Disabled
@Autonomous(name = "File Write Test", group = "File")
public class FileWriteTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //Init here

        telemetry.addLine("Status: Initialized and ready!");
        telemetry.update();

        waitForStart();

        try {
            File file = new File("/storage/self/primary/", "LastTeamColor.txt");
            FileOutputStream outStream = new FileOutputStream(file);
            outStream.write(3);
            outStream.flush();
            outStream.close();
        } catch (Exception e) {
            telemetry.addLine(e.toString());
            telemetry.update();
        }
    }
}