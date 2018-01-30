/*package org.firstinspires.ftc.teamcode.MainBot.teleop.DriveAssembly.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

@Disabled
@Autonomous(name = "ControlTest", group = "Test")
public class ControlTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //Init here

        telemetry.addLine("Status: Initialized and ready!");
        for (Driver d : hardwareMap.getAll(Driver.class)) {
            telemetry.addLine(d.getDeviceName());
        }
        telemetry.update();

        waitForStart();

        // Do something useful

    }
}*/