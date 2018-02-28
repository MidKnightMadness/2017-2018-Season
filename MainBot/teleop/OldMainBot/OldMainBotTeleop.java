package org.firstinspires.ftc.teamcode.MainBot.teleop.OldMainBot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "Main Bot", group = "Main Bot")
public class OldMainBotTeleop extends OpMode {

    private OldMainBotController controller;

    public OldMainBotTeleop() {
        super();
        controller = new OldMainBotController();
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
