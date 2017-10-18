package org.firstinspires.ftc.teamcode.MainBot.teleop.JewelDriveBot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MainBot.teleop.SimpleDriveAssembly.SimpleDriveAssemblyController;
import org.firstinspires.ftc.teamcode.MainBot.teleop.JewelAssembly.JewelAssemblyController;

/**
 * Created by nisha.prasad on 10/17/17.
 */

public class JewelDriveBotController {

    private Telemetry telemetry;
    private SimpleDriveAssemblyController simpleDriveAssemblyController;
    private JewelAssemblyController jewelAssemblyController;

    public JewelDriveBotController() {
        jewelAssemblyController = new JewelAssemblyController();
        simpleDriveAssemblyController = new SimpleDriveAssemblyController();
    }

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        jewelAssemblyController.init(telemetry, hardwareMap);
        simpleDriveAssemblyController.init(telemetry, hardwareMap);

    }

    public void start() {
        jewelAssemblyController.start();
        simpleDriveAssemblyController.start();

    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        jewelAssemblyController.loop(gamepad1, gamepad2);
        simpleDriveAssemblyController.loop(gamepad1, gamepad2);

    }

    public void stop() {
        jewelAssemblyController.stop();
        simpleDriveAssemblyController.stop();

    }
}