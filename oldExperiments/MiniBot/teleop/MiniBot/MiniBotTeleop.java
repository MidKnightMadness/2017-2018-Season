package org.firstinspires.ftc.teamcode.MiniBot.teleop.MiniBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Mini Bot", group = "Mini Bot")
public class MiniBotTeleop extends OpMode {

    private MiniBotController controller;

    public MiniBotTeleop() {
        super();
        controller = new MiniBotController();
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
