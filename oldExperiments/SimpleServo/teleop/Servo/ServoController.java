package org.firstinspires.ftc.teamcode.SimpleServo.teleop.Servo;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SimpleServo.teleop.CrossCommunicator;

/**
 * Created by gregory.ling on 7/28/17.
 */

public class ServoController {
    private Servo servo;
    private Servo servo2;
    private Telemetry telemetry;

    private void log(String data) {
        telemetry.addLine(data);
    }

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        servo = hardwareMap.servo.get(CrossCommunicator.Servo.SERVO);
        servo.setPosition(0);
        servo2 = hardwareMap.servo.get("servo2");
        servo2.setPosition(0);
    }

    public void start() {

    }

    private double pos = 0;
    private double dir = 0.1;
    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        if (pos < 1) {
            pos += dir;
            servo2.setPosition(pos);
            log("" + pos);
        } else if (pos < 2){
            pos += dir;
            servo.setPosition(pos - 1);
            log("" + pos);
        }
    }

    public void stop() {

    }
}
