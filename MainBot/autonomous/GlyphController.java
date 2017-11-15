package org.firstinspires.ftc.teamcode.MainBot.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

public class GlyphController {

    private Telemetry telemetry;
    private DcMotor motor;
    private Servo servo;

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        motor = hardwareMap.dcMotor.get(CrossCommunicator.Glyph.MOTOR);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo = hardwareMap.servo.get(CrossCommunicator.Glyph.SERVO);
        servo.setPosition(1);
    }

    public void start() {

    }

    public void stop() {

    }

    public void grab() {
        servo.setPosition(0);
    }

    public void lift() {
        motor.setPower(1);
    }

    public void drop() {
        servo.setPosition(1);
    }

}
