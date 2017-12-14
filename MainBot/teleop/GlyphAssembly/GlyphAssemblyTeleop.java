package org.firstinspires.ftc.teamcode.MainBot.teleop.GlyphAssembly;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

@Disabled
@TeleOp(name = "Main Bot Glyph", group = "Main Bot")
public class GlyphAssemblyTeleop extends OpMode {

    private GlyphAssemblyController controller;

    public GlyphAssemblyTeleop() {
        super();
        controller = new GlyphAssemblyController();
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