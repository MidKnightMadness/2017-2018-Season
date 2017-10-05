package org.firstinspires.ftc.teamcode.MainBot.teleop.ElevatorAssembly;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Main Bot Elevator", group = "Main Bot")
public class ElevatorTeleop extends OpMode {

    private ElevatorController controller;

    public ElevatorTeleop() {
        super();
        controller = new ElevatorController();
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
