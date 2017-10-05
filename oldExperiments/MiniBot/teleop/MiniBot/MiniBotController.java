package org.firstinspires.ftc.teamcode.MiniBot.teleop.MiniBot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MiniBot.teleop.DriveAssembly.DriveController;

public class MiniBotController {

    private DriveController driveController;

    public MiniBotController() {
        driveController = new DriveController();
    }

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        driveController.init(telemetry, hardwareMap);
    }

    public void start() {
        driveController.start();
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        driveController.loop(gamepad1, gamepad2);
    }

    public void stop() {
        driveController.stop();
    }
}
