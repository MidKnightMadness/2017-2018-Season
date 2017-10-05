package org.firstinspires.ftc.teamcode.MainBot.teleop.DriveAssembly;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Main Bot Drive", group = "Main Bot")
public class DriveTeleop extends OpMode {

    private DriveController controller;

    public DriveTeleop() {
        super();
        controller = new DriveController();
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
