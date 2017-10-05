package org.firstinspires.ftc.teamcode.MainBot.teleop.MainBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Main Bot", group = "Main Bot")
public class MainBotTeleop extends OpMode {

    private MainBotController controller;

    public MainBotTeleop() {
        super();
        controller = new MainBotController();
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
