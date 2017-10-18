package org.firstinspires.ftc.teamcode.MainBot.teleop.JewelAssembly;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

public class JewelAssemblyNewController {
    private DcMotor motor;
    private Telemetry telemetry;
    boolean pressed = false;
    double power = 0.3;
    int distance = 529;

    private void log(String data) {
        telemetry.addLine(data);
    }

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        motor = hardwareMap.dcMotor.get(CrossCommunicator.Jewel.MOTOR);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void start() {}

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {

        if (gamepad1.x && !pressed && (!motor.isBusy())) {
            down();
            pressed = true;
        }
        if (pressed && (!motor.isBusy())) {
            up();
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

    public void down() {
        motor.setTargetPosition(motor.getCurrentPosition() - distance);
        motor.setPower(power);
    }

    public void up() {
        motor.setTargetPosition(motor.getCurrentPosition() + distance);
        motor.setPower(-power);
    }

    public void update() {}

    public boolean isBusy() {
        return motor.isBusy();
    }

    public void stop() {

    }
}
