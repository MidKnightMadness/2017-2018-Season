package org.firstinspires.ftc.teamcode.MainBot.teleop.JewelDriveBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by nisha.prasad on 10/17/17.
 */
@TeleOp(name = "Jewel Drive Bot", group = "Main Bot")
public class JewelDriveBotTeleop  extends OpMode {

    private JewelDriveBotController controller;

    public JewelDriveBotTeleop() {
        super();
        controller = new JewelDriveBotController();
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
