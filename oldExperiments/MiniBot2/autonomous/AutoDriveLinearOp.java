package org.firstinspires.ftc.teamcode.MiniBot2.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.MiniBot.teleop.CrossCommunicator;

@Autonomous(name = "Mini Bot Delay Move2", group = "Mini Bot2")
public class AutoDriveLinearOp extends LinearOpMode {
    double waitUntil = 0;

    final double getTimeLeft() {
        return time - waitUntil;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Status: Initialized and ready!");
        telemetry.update();

        telemetry.addData("Time Left: ", new Func<Integer>() {
            @Override
            public Integer value() {
                return (int)getTimeLeft();
            }
        });



        waitForStart();

    }
}