package org.firstinspires.ftc.teamcode.MainBot.teleop.FeederAssembly;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

public class FeederController {

    private static final double SERVO_POSITION_HORIZONTAL = 0;
    private static final double SERVO_POSITION_VERTICAL = 0.5;
    private Servo servo;

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        servo = hardwareMap.servo.get(CrossCommunicator.Feeder.SERVO);
        servo.setPosition(SERVO_POSITION_HORIZONTAL);
    }

    public void start() {
        servo.setPosition(SERVO_POSITION_VERTICAL);
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
    }

    public void stop() {
    }
}
