package org.firstinspires.ftc.teamcode.MainBot.teleop.DriveAssembly.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

@Autonomous(name = "Encoder Testing", group = "Main Bot")
public class EncoderTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //Init here
        DcMotor motorUp;
        DcMotor motorDown;
        DcMotor motorLeft;
        DcMotor motorRight;

        telemetry.addLine("Status: Initialized and ready!");
        telemetry.update();
        motorUp = hardwareMap.dcMotor.get(CrossCommunicator.Drive.FRONT_LEFT);
        motorUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDown = hardwareMap.dcMotor.get(CrossCommunicator.Drive.BACK_RIGHT);
        motorDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft = hardwareMap.dcMotor.get(CrossCommunicator.Drive.BACK_LEFT);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight = hardwareMap.dcMotor.get(CrossCommunicator.Drive.FRONT_RIGHT);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        motorUp.setPower(1);
        while (time < 5) {
            telemetry.addData("Motor Up", motorUp.getCurrentPosition());
            telemetry.update();
            idle();
        }
        motorUp.setPower(0);
        motorLeft.setPower(1);
        while (time < 10) {
            telemetry.addData("Motor Left", motorLeft.getCurrentPosition());
            telemetry.update();
            idle();
        }
        motorLeft.setPower(0);
        motorDown.setPower(1);
        while (time < 15) {
            telemetry.addData("Motor Down", motorDown.getCurrentPosition());
            telemetry.update();
            idle();
        }
        motorDown.setPower(0);
        motorRight.setPower(1);
        while (time < 20 ) {
            telemetry.addData("Motor Right", motorRight.getCurrentPosition());
            telemetry.update();
            idle();
        }
        motorRight.setPower(0);




        /*while (time < 20) {
            telemetry.addData("Up Motor (0)", motorUp.getCurrentPosition());
            telemetry.addData("Left Motor (1)", motorDown.getCurrentPosition());
            telemetry.addData("Down Motor (2)", motorLeft.getCurrentPosition());
            telemetry.addData("Right Motor (3)", motorRight.getCurrentPosition());
            telemetry.update();
            idle();
        }*/
        // Do something useful

    }
}