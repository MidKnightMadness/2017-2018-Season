package org.firstinspires.ftc.teamcode.MainBot.teleop.MainBot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MainBot.teleop.DriveAssembly.DriveAssemblyController;
import org.firstinspires.ftc.teamcode.MainBot.teleop.GlyphAssembly.GlyphAssemblyController;
import org.firstinspires.ftc.teamcode.MainBot.teleop.JewelAssembly.JewelAssemblyController;

/**
 * Created by nisha.prasad on 10/17/17.
 */

public class MainBotController {

    private Telemetry telemetry;
    private DriveAssemblyController driveAssemblyController;
    private JewelAssemblyController jewelAssemblyController;
    private GlyphAssemblyController glyphAssemblyController;

    public MainBotController() {
        jewelAssemblyController = new JewelAssemblyController();
        glyphAssemblyController = new GlyphAssemblyController();
        driveAssemblyController = new DriveAssemblyController();
    }

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        jewelAssemblyController.init(telemetry, hardwareMap);
        glyphAssemblyController.init(telemetry, hardwareMap);
        driveAssemblyController.init(telemetry, hardwareMap);
    }

    public void start() {
        jewelAssemblyController.start();
        glyphAssemblyController.start();
        driveAssemblyController.start();
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        jewelAssemblyController.loop(gamepad1, gamepad2);
        glyphAssemblyController.loop(gamepad1, gamepad2);
        driveAssemblyController.loop(gamepad1, gamepad2);
    }

    public void stop() {
        jewelAssemblyController.stop();
        glyphAssemblyController.stop();
        driveAssemblyController.stop();
    }
}