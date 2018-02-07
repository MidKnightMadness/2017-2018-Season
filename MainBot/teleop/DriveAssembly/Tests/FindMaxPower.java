package org.firstinspires.ftc.teamcode.MainBot.teleop.DriveAssembly.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

@Disabled
@Autonomous(name = "Drive Test: Find Max Power", group = "Main Bot")
public class FindMaxPower extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor motorUp;
        double speeds[] = new double[22];

        motorUp = hardwareMap.dcMotor.get(CrossCommunicator.Drive.FRONT_LEFT);
        motorUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Status: Initialized and ready!");
        telemetry.update();

        waitForStart();

        double startTime;
        double startPos;
        double waitUntil;

        for (double i = 1; i >= -1; i -= 0.1) {
            motorUp.setPower(i);

            waitUntil = time + 1;
            while (time < waitUntil) {
                telemetry.addLine("Waiting...");
                telemetry.update();
                idle();
            }
            telemetry.addLine("Waiting2...");
            telemetry.update();

            startTime = time;
            startPos = motorUp.getCurrentPosition();

            waitUntil = time + 4;
            while (time < waitUntil) {
                telemetry.addLine("Waiting3...");
                telemetry.addData("Speed Test " + i + ": ", (motorUp.getCurrentPosition() - startPos) / (time - startTime));
                telemetry.update();
                idle();
            }
            speeds[20 - ((int) ((i+1)*10))] = (motorUp.getCurrentPosition() - startPos) / (time - startTime);
        }

        motorUp.setPower(0);

        waitUntil = time + 1;
        while (time < waitUntil) {
            idle();
        }

        motorUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorUp.setPower(1);

        waitUntil = time + 1;
        while (time < waitUntil) {
            idle();
        }

        startTime = time;
        startPos = motorUp.getCurrentPosition();

        waitUntil = time + 4;
        while (time < waitUntil) {
            telemetry.addLine("Speed Test Max Motor Power: " + (motorUp.getCurrentPosition() - startPos) / (time - startTime));
            telemetry.update();
            idle();
        }
        speeds[21] = (motorUp.getCurrentPosition() - startPos) / (time - startTime);

        motorUp.setPower(0);

        for (double i = 1; i >= -1; i -= 0.1) {
            telemetry.addLine("Speed Test " + i + ": " + speeds[(int) (20 - (i+1)*10)]);
        }
        telemetry.addLine("Speed Test Max Motor Power: " + speeds[21]);
        telemetry.update();
        while (time < 1200) {
            idle();
        }
    }
}