package org.firstinspires.ftc.teamcode.MainBot.teleop.MainBot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MainBot.teleop.DriveAssembly.DriveAssemblyController;
import org.firstinspires.ftc.teamcode.MainBot.teleop.SimpleDriveAssembly.SimpleDriveAssemblyController;
import org.firstinspires.ftc.teamcode.MainBot.teleop.JewelAssembly.JewelAssemblyController;

/**
 * Created by nisha.prasad on 10/17/17.
 */

public class MainBotController {

    private Telemetry telemetry;
    private DriveAssemblyController driveAssemblyController;
    private JewelAssemblyController jewelAssemblyController;

    public MainBotController() {
        jewelAssemblyController = new JewelAssemblyController();
        driveAssemblyController = new DriveAssemblyController();
    }

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        jewelAssemblyController.init(telemetry, hardwareMap);
        driveAssemblyController.init(telemetry, hardwareMap);
    }

    public void start() {
        jewelAssemblyController.start();
        driveAssemblyController.start();
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        jewelAssemblyController.loop(gamepad1, gamepad2);
        driveAssemblyController.loop(gamepad1, gamepad2);
    }

    public void stop() {
        jewelAssemblyController.stop();
        driveAssemblyController.stop();
    }
}