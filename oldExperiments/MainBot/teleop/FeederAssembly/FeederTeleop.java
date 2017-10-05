package org.firstinspires.ftc.teamcode.MainBot.teleop.FeederAssembly;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Main Bot Feeder", group = "Main Bot")
public class FeederTeleop extends OpMode {

    private FeederController controller;

    public FeederTeleop() {
        super();
        controller = new FeederController();
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
