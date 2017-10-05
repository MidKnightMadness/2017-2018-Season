package org.firstinspires.ftc.teamcode.MainBot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

@Autonomous(name = "Delay Move")
public class AutoDriveLinearOp extends LinearOpMode {

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.dcMotor.get(CrossCommunicator.Drive.MOTOR_FRONT_LEFT);
        frontLeftMotor.resetDeviceConfigurationForOpMode();
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRightMotor = hardwareMap.dcMotor.get(CrossCommunicator.Drive.MOTOR_FRONT_RIGHT);
        frontRightMotor.resetDeviceConfigurationForOpMode();
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);

        backLeftMotor = hardwareMap.dcMotor.get(CrossCommunicator.Drive.MOTOR_BACK_LEFT);
        backLeftMotor.resetDeviceConfigurationForOpMode();
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backRightMotor = hardwareMap.dcMotor.get(CrossCommunicator.Drive.MOTOR_BACK_RIGHT);
        backRightMotor.resetDeviceConfigurationForOpMode();
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized and ready!");
        telemetry.update();

        waitForStart();

        double waitUntil = time + 5;
        while(time < waitUntil){
            telemetry.addData("Time left", (waitUntil - time));
            telemetry.update();
            idle();
        }

        // Drive forward for 5 seconds
        frontLeftMotor.setPower(1);
        frontRightMotor.setPower(1);
        backLeftMotor.setPower(1);
        backRightMotor.setPower(1);

        waitUntil = time + 1;
        while(time < waitUntil)
            idle();
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
