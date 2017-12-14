package org.firstinspires.ftc.teamcode.MainBot.teleop.JewelAssembly;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "Main Bot Jewel", group = "Main Bot")
public class JewelAssemblyTeleop extends OpMode {

    private JewelAssemblyController controller;

    public JewelAssemblyTeleop() {
        super();
        controller = new JewelAssemblyController();
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