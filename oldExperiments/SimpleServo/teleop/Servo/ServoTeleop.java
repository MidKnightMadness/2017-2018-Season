package org.firstinspires.ftc.teamcode.SimpleServo.teleop.Servo;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Simple Servo", group = "Simple Servo")
public class ServoTeleop extends OpMode {

    private ServoController controller;

    public ServoTeleop() {
        super();
        controller = new ServoController();
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