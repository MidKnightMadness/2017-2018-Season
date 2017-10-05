package org.firstinspires.ftc.teamcode.OmniBot.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OmniBot.teleop.CrossCommunicator;

@TeleOp(name = "Omni Bot Drive Testing", group = "OmniBot")
public class DriveTestingTeleop extends OpMode {

    private DriveTestingController controller;

    public DriveTestingTeleop() {
        super();
        controller = new DriveTestingController();
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