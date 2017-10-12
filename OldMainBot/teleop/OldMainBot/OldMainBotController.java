package org.firstinspires.ftc.teamcode.OldMainBot.teleop.OldMainBot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OldMainBot.teleop.DriveAssembly.DriveController;
import org.firstinspires.ftc.teamcode.OldMainBot.teleop.ElevatorAssembly.ElevatorController;
import org.firstinspires.ftc.teamcode.OldMainBot.teleop.FeederAssembly.FeederController;
import org.firstinspires.ftc.teamcode.OldMainBot.teleop.IntakeAssembly.IntakeController;
import org.firstinspires.ftc.teamcode.OldMainBot.teleop.PinballAssembly.PinballController;

public class OldMainBotController {

    private PinballController pinballController;
    private IntakeController intakeController;
    private ElevatorController elevatorController;
    private DriveController driveController;
    private FeederController feederController;

    public OldMainBotController() {
        pinballController = new PinballController();
        intakeController = new IntakeController();
        elevatorController = new ElevatorController();
        driveController = new DriveController();
        feederController = new FeederController();
    }

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        pinballController.init(telemetry, hardwareMap);
        intakeController.init(telemetry, hardwareMap);
        elevatorController.init(telemetry, hardwareMap);
        driveController.init(telemetry, hardwareMap);
        feederController.init(telemetry, hardwareMap);
    }

    public void start() {
        pinballController.start();
        intakeController.start();
        elevatorController.start();
        driveController.start();
        feederController.start();
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        pinballController.loop(gamepad1, gamepad2);
        intakeController.loop(gamepad1, gamepad2);
        elevatorController.loop(gamepad1, gamepad2);
        driveController.loop(gamepad1, gamepad2);
        feederController.loop(gamepad1, gamepad2);
    }

    public void stop() {
        pinballController.stop();
        intakeController.stop();
        elevatorController.stop();
        driveController.stop();
        feederController.stop();
    }
}