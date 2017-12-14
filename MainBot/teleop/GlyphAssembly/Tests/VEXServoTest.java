package org.firstinspires.ftc.teamcode.MainBot.teleop.GlyphAssembly.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
@TeleOp(name = "VEX Servo Test", group = "Tests")
public class VEXServoTest extends OpMode {

    private Servo servo;

    public void init() {
        servo = hardwareMap.servo.get("servo");
    }

    public void start() {}

    public void loop() {
        if (gamepad1.left_trigger != 0) {
            servo.setDirection(Servo.Direction.REVERSE);
            servo.setPosition(0);
        } else if (gamepad1.right_trigger != 0) {
            servo.setDirection(Servo.Direction.FORWARD);
            servo.setPosition(0);
        } else {
            servo.setPosition(0.5);
        }
        telemetry.update();
    }

    public void stop() {}
}