package org.firstinspires.ftc.teamcode.MainBot.autonomous.JewelAssembly;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

public class JewelAssemblyAutonomousController {
    private DcMotor motor;
    double power = 0.3;
    int distance = 529;

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        motor = hardwareMap.dcMotor.get(CrossCommunicator.Jewel.MOTOR);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void down() {
        motor.setTargetPosition(motor.getCurrentPosition() - distance);
        motor.setPower(power);
    }

    public void up() {
        motor.setTargetPosition(motor.getCurrentPosition() + distance);
        motor.setPower(-power);
    }

    public boolean isBusy() {
        return motor.isBusy();
    }
}
