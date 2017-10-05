package org.firstinspires.ftc.teamcode.MiniBot.teleop.CircularDriveAssembly;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Mini Bot Circular Drive", group = "Mini Bot")
public class CircularDriveTeleop extends OpMode {

    private CircularDriveController controller;

    public CircularDriveTeleop() {
        super();
        controller = new CircularDriveController();
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
