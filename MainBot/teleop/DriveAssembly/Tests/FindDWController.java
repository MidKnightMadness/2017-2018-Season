package org.firstinspires.ftc.teamcode.MainBot.teleop.DriveAssembly.Tests;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

public class FindDWController {

    private Telemetry telemetry;
    private DcMotor motorUp;
    private DcMotor motorDown;
    private DcMotor motorLeft;
    private DcMotor motorRight;


    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        motorUp = hardwareMap.dcMotor.get(CrossCommunicator.Drive.FRONT_LEFT);
        motorUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDown = hardwareMap.dcMotor.get(CrossCommunicator.Drive.BACK_RIGHT);
        motorDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft = hardwareMap.dcMotor.get(CrossCommunicator.Drive.BACK_LEFT);
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight = hardwareMap.dcMotor.get(CrossCommunicator.Drive.FRONT_RIGHT);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void start() {
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {

        motorUp.setPower(gamepad1.right_stick_x * 0.5);
        motorDown.setPower(gamepad1.right_stick_x * 0.5);
        motorLeft.setPower(gamepad1.right_stick_x * 0.5);
        motorRight.setPower(gamepad1.right_stick_x * 0.5);

        telemetry.addData("Motor Up: ", motorUp.getCurrentPosition());
        telemetry.addData("Motor Down: ", motorDown.getCurrentPosition());
        telemetry.addData("Motor Left: ", motorLeft.getCurrentPosition());
        telemetry.addData("Motor Right: ", motorRight.getCurrentPosition());

        telemetry.update();
    }

    public void stop() {

    }
}
