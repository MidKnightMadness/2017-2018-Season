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

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        motor = hardwareMap.dcMotor.get(CrossCommunicator.Glyph.MOTOR);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servo = hardwareMap.servo.get(CrossCommunicator.Glyph.SERVO);
        servo.setPosition(1);
        open = true;
    }

    public void start() {

    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {

        if (gamepad1.right_bumper) {
            motor.setPower(1);
        } else if (gamepad1.left_bumper) {
            motor.setPower(-1);
        } else {
            motor.setPower(0);
        }


        if (gamepad1.dpad_left) {
            servo.setPosition(-1);
        }
        else if (gamepad1.dpad_right){
                servo.setPosition(1);
        }


        telemetry.addLine("Servo Position: " + servo.getPosition());

    }

    public void stop() {

    }
}
