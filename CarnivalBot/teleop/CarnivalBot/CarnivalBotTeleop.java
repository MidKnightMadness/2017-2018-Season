package org.firstinspires.ftc.teamcode.CarnivalBot.teleop.CarnivalBot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "Carnival Bot", group = "Carnival Bot")
public class CarnivalBotTeleop extends OpMode {

    private CarnivalBotController controller;

    public CarnivalBotTeleop() {
        super();
        controller = new CarnivalBotController();
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
