package org.firstinspires.ftc.teamcode.MiniBot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MiniBot.teleop.CrossCommunicator;

@Autonomous(name = "Mini Bot Delay Move", group = "Mini Bot")
public class AutoDriveLinearOp extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get(CrossCommunicator.Drive.MOTOR_LEFT);
        leftMotor.resetDeviceConfigurationForOpMode();

        rightMotor = hardwareMap.dcMotor.get(CrossCommunicator.Drive.MOTOR_RIGHT);
        rightMotor.resetDeviceConfigurationForOpMode();
        //rightMotor.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Status", "Initialized and ready!");
        telemetry.update();

        waitForStart();

        double waitUntil = time + 5;
        while(time < waitUntil){
            telemetry.addData("Time left", (waitUntil - time));
            telemetry.update();
            idle();
        }
        telemetry.addLine("BOOM!");

        // Drive forward for 5 seconds
        leftMotor.setPower(1);
        rightMotor.setPower(1);

        waitUntil = time + 3;
        while(time < waitUntil)
            idle();
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
