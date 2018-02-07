package org.firstinspires.ftc.teamcode.MainBot.teleop.DriveAssembly.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

@Disabled
@Autonomous(name = "Drive Test: Find Max Power2", group = "Main Bot")
public class FindMaxPower2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor motorUp;
        double speeds[] = new double[22];

        motorUp = hardwareMap.dcMotor.get(CrossCommunicator.Drive.FRONT_LEFT);
        motorUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        telemetry.addLine("Status: Initialized and ready!");
        telemetry.update();

        waitForStart();

        /*for (int i = 0; i < 1000; i++) {
            double waitUntil = time + 0.05;
            while (time < waitUntil) {
                idle();
            }
            telemetry.addLine(i + "");
            telemetry.addLine(motorUp.getCurrentPosition() + "");
            telemetry.update();
            motorUp.setTargetPosition(motorUp.getCurrentPosition() + (int) (130 * (Math.sin(i*(Math.PI/180)))));
            motorUp.setPower(1);
        }*/

        motorUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        for (int i = 0; i < 1000; i++) {
            double waitUntil = time + 0.05;
            while (time < waitUntil) {
                idle();
            }
            motorUp.setPower(0); //Math.sin(Math.sin(i*(Math.PI/180))));
        }



    }
}