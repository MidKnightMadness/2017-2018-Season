package org.firstinspires.ftc.teamcode.MainBot.teleop.GlyphAssembly;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

public class GlyphAssemblyController {

    private Telemetry telemetry;
    private DcMotor motor;
    private Servo servo;
    private boolean open;
    private int pos;
    private double servoPos;

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        motor = hardwareMap.dcMotor.get(CrossCommunicator.Glyph.MOTOR);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo = hardwareMap.servo.get(CrossCommunicator.Glyph.SERVO);
        servo.setPosition(1);
        open = true;
        pos = motor.getCurrentPosition();
        servoPos = 1;
    }

    public void start() {

    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        telemetry.addData("Elevator", pos - motor.getCurrentPosition());
        if (gamepad1.right_bumper) {
            motor.setPower((motor.getCurrentPosition() - pos < 10300) ? 1 : 0);
        } else if (gamepad1.left_bumper) {
            motor.setPower((motor.getCurrentPosition() - pos > 0) ? -1 : 0);
        } else {
            motor.setPower(0);
        }


        if (gamepad1.dpad_left) {
            if (servoPos < 0) {
                servoPos = 0;
            } else {
                servoPos -= 0.1;
            }
        } else if (gamepad1.dpad_right){
            if (servoPos > 1) {
                servoPos = 1;
            } else {
                servoPos += 0.1;
            }
        }

        telemetry.addData("Servo", servoPos);
        servo.setPosition(servoPos);


        telemetry.addLine("Servo Position: " + servo.getPosition());

    }

    public void stop() {

    }
}
