package org.firstinspires.ftc.teamcode.MainBot.teleop.DriveAssembly;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "Main Bot Drive", group = "Main Bot")
public class DriveAssemblyTeleop extends OpMode {

    private DriveAssemblyController controller;

    public DriveAssemblyTeleop() {
        super();
        controller = new DriveAssemblyController();
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