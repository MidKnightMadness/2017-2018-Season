package org.firstinspires.ftc.teamcode.MainBot.teleop.SimpleDriveAssembly;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by nisha.prasad on 10/17/17.
 */

@Disabled
@TeleOp(name = "Simple Drive", group = "Main Bot")
public class SimpleDriveAssemblyTeleop extends OpMode {

    private SimpleDriveAssemblyController controller;

    public SimpleDriveAssemblyTeleop() {
        super();
        controller = new SimpleDriveAssemblyController();
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
