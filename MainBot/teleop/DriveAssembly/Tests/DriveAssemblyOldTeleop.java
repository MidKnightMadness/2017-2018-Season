package org.firstinspires.ftc.teamcode.MainBot.teleop.DriveAssembly.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "Main Bot Old Drive", group = "Main Bot")
public class DriveAssemblyOldTeleop extends OpMode {

    private DriveAssemblyNotSoOldController controller;

    public DriveAssemblyOldTeleop() {
        super();
        controller = new DriveAssemblyNotSoOldController();
    }

    @Override
    public void init() {
        controller.init(telemetry, hardwareMap);
    }

    @Override
    public void start() {
        controller.start();
    }

    @Override
    public void loop() {
        controller.loop(gamepad1, gamepad2);

        telemetry.update();
    }

    @Override
    public void stop() {
        controller.stop();
    }
}