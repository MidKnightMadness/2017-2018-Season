package org.firstinspires.ftc.teamcode.MainBot.autonomous.Tests.OlderTests;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

public class GlyphController {

    private Telemetry telemetry;
    private DcMotor motor;
    private Servo servo;
    private int pos;

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        motor = hardwareMap.dcMotor.get(CrossCommunicator.Glyph.ELEV);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pos = motor.getCurrentPosition();

        servo = hardwareMap.servo.get(CrossCommunicator.Glyph.GRAB_UPPER);
        servo.setPosition(1);
    }

    public void start() {

    }

    public void stop() {

    }

    public void close() {
        servo.setPosition(0);
    }

    public void open() {
        servo.setPosition(0.6);
    }

    public void lift() {
        motor.setTargetPosition(pos + 1000);
        motor.setPower(1);
    }

    public void lower() {
        motor.setTargetPosition(pos);
        motor.setPower(-1);
    }

    public void resetArm() {
        servo.setPosition(0.6);
        //motor.setTargetPosition(pos - 2000);
        //motor.setPower(-1);
    }
}
