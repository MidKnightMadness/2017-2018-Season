package org.firstinspires.ftc.teamcode.OldMainBot.teleop.PinballAssembly;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "Main Bot Pinball", group = "Main Bot")
public class PinballTeleop extends OpMode {

    private PinballController controller;

    public PinballTeleop() {
        super();
        controller = new PinballController();
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