package org.firstinspires.ftc.teamcode.MainBot.teleop.JewelAssembly;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

/**
 * Created by gregory.ling on 7/28/17.
 */

public class JewelAssemblyController {
    private DcMotor motor;
    private Telemetry telemetry;
    private ElapsedTime runtime;
    private double waitUntil = 0;
    boolean pressed = false;
    double power = .3;
    int distance = 529;

    private void log(String data) {
        telemetry.addLine(data);
    }

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        motor = hardwareMap.dcMotor.get(CrossCommunicator.Jewel.MOTOR);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }

    public void start() {
        runtime.reset();
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {

        if (gamepad1.x && !pressed && (runtime.time() > waitUntil)) {
            pressed = true;
            motor.setTargetPosition(motor.getCurrentPosition() - distance);
            motor.setPower(power);

            waitUntil = runtime.time() + 2;
        }
        if (pressed && (runtime.time() > waitUntil)) {
            motor.setTargetPosition(motor.getCurrentPosition() + distance);
            motor.setPower(-power);
            waitUntil = runtime.time() + 2;
            pressed = false;
        }
        if (gamepad1.dpad_down) {
            motor.setTargetPosition(motor.getCurrentPosition() - 10);
            motor.setPower(power);
        }

        if (gamepad1.dpad_up) {
            motor.setTargetPosition(motor.getCurrentPosition() + 10);
            motor.setPower(-power);
        }
    }

    public void stop() {

    }
}
