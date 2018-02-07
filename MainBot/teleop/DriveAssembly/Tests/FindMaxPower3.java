package org.firstinspires.ftc.teamcode.MainBot.teleop.DriveAssembly.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

@Disabled
@Autonomous(name = "Drive Test: Find Max Power3", group = "Main Bot")
public class FindMaxPower3 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor motorUp;
        DcMotor motorDown;
        DcMotor motorLeft;
        DcMotor motorRight;


        double speeds[][] = new double[4][22];

        motorUp = hardwareMap.dcMotor.get(CrossCommunicator.Drive.FRONT_LEFT);
        motorUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDown = hardwareMap.dcMotor.get(CrossCommunicator.Drive.BACK_RIGHT);
        motorDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft = hardwareMap.dcMotor.get(CrossCommunicator.Drive.BACK_LEFT);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight = hardwareMap.dcMotor.get(CrossCommunicator.Drive.FRONT_RIGHT);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addLine("Status: Initialized and ready!");
        telemetry.update();

        waitForStart();

        double startTime;
        double startPos[] = new double[4];
        double waitUntil;

        for (double i = 1; i >= -1; i -= 0.1) {
            motorUp.setPower(i);
            motorDown.setPower(i);
            motorLeft.setPower(i);
            motorRight.setPower(i);

            waitUntil = time + 1;
            while (time < waitUntil) {
                telemetry.addLine("Waiting to get to Speed...");
                telemetry.update();
                idle();
            }

            startTime = time;
            startPos[0] = motorUp.getCurrentPosition();
            startPos[1] = motorDown.getCurrentPosition();
            startPos[2] = motorLeft.getCurrentPosition();
            startPos[3] = motorRight.getCurrentPosition();

            waitUntil = time + 4;
            while (time < waitUntil) {
                telemetry.addData("Speed Test " + i + ": ", (motorUp.getCurrentPosition() - startPos[0]) / (time - startTime));
                telemetry.update();
                idle();
            }
            speeds[0][20 - ((int) ((i+1)*10))] = (motorUp.getCurrentPosition() - startPos[0]) / (time - startTime);
            speeds[1][20 - ((int) ((i+1)*10))] = (motorDown.getCurrentPosition() - startPos[1]) / (time - startTime);
            speeds[2][20 - ((int) ((i+1)*10))] = (motorLeft.getCurrentPosition() - startPos[2]) / (time - startTime);
            speeds[3][20 - ((int) ((i+1)*10))] = (motorRight.getCurrentPosition() - startPos[3]) / (time - startTime);
        }

        motorUp.setPower(0);
        motorDown.setPower(0);
        motorLeft.setPower(0);
        motorRight.setPower(0);

        waitUntil = time + 1;
        while (time < waitUntil) {
            idle();
        }

        motorUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorUp.setPower(1);
        motorDown.setPower(1);
        motorLeft.setPower(1);
        motorRight.setPower(1);

        waitUntil = time + 1;
        while (time < waitUntil) {
            idle();
        }

        startTime = time;
        startPos[0] = motorUp.getCurrentPosition();
        startPos[1] = motorDown.getCurrentPosition();
        startPos[2] = motorLeft.getCurrentPosition();
        startPos[3] = motorRight.getCurrentPosition();

        waitUntil = time + 4;
        while (time < waitUntil) {
            telemetry.addLine("Speed Test Max Motor Power: " + (motorUp.getCurrentPosition() - startPos[0]) / (time - startTime));
            telemetry.update();
            idle();
        }
        speeds[0][21] = (motorUp.getCurrentPosition() - startPos[0]) / (time - startTime);
        speeds[1][21] = (motorDown.getCurrentPosition() - startPos[1]) / (time - startTime);
        speeds[2][21] = (motorLeft.getCurrentPosition() - startPos[2]) / (time - startTime);
        speeds[3][21] = (motorRight.getCurrentPosition() - startPos[3]) / (time - startTime);

        motorUp.setPower(0);

        String speedString = "";
        for (double i = 1; i >= -1; i -= 0.1) {
            speedString += "Speed Test " + i + " M0: " + speeds[0][(int) (20 - (i+1)*10)] + "\n" + "M1: " + speeds[1][(int) (20 - (i+1)*10)] + "\n" + "M2: " + speeds[2][(int) (20 - (i+1)*10)] + "\n" + "M3: " + speeds[3][(int) (20 - (i+1)*10)] + "\n";
        }

        speedString += "Speed Test Max Motor Power M0: " + speeds[0][21] + "\n" + "M1: " + speeds[1][21] + "\n" + "M2: " + speeds[2][21] + "\n" + "M3: " + speeds[3][21] + "\n";

        telemetry.addLine(speedString);
        telemetry.update();
        while (time < 1200) {
            idle();
        }
    }
}