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
        DcMotor motorFL;
        DcMotor motorBR;
        DcMotor motorBL;
        DcMotor motorFR;

        telemetry.addLine("Status: Initialized and ready!");
        telemetry.update();
        motorFL = hardwareMap.dcMotor.get(CrossCommunicator.Drive.FRONT_LEFT);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR = hardwareMap.dcMotor.get(CrossCommunicator.Drive.BACK_RIGHT);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL = hardwareMap.dcMotor.get(CrossCommunicator.Drive.BACK_LEFT);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR = hardwareMap.dcMotor.get(CrossCommunicator.Drive.FRONT_RIGHT);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        motorFL.setPower(1);
        while (time < 5) {
            telemetry.addData("Motor FrontLeft", motorFL.getCurrentPosition());
            telemetry.update();
            idle();
        }
        motorFL.setPower(0);
        motorBL.setPower(1);
        while (time < 10) {
            telemetry.addData("Motor BackLeft", motorBL.getCurrentPosition());
            telemetry.update();
            idle();
        }
        motorBL.setPower(0);
        motorBR.setPower(1);
        while (time < 15) {
            telemetry.addData("Motor BackRight", motorBR.getCurrentPosition());
            telemetry.update();
            idle();
        }
        motorBR.setPower(0);
        motorFR.setPower(1);
        while (time < 20 ) {
            telemetry.addData("Motor FrontRight", motorFR.getCurrentPosition());
            telemetry.update();
            idle();
        }
        motorFR.setPower(0);




        /*while (time < 20) {
            telemetry.addData("Up Motor (0)", motorFL.getCurrentPosition());
            telemetry.addData("Left Motor (1)", motorBR.getCurrentPosition());
            telemetry.addData("Down Motor (2)", motorLeft.getCurrentPosition());
            telemetry.addData("Right Motor (3)", motorRight.getCurrentPosition());
            telemetry.update();
            idle();
        }*/
        // Do something useful

    }
}