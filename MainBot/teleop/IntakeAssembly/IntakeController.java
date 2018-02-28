package org.firstinspires.ftc.teamcode.MainBot.teleop.IntakeAssembly;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

public class IntakeController {

    private DcMotor motor;

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        motor = hardwareMap.dcMotor.get(CrossCommunicator.Intake.MOTOR);
        motor.resetDeviceConfigurationForOpMode();
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Intake Power", new Func<Double>() {
            @Override
            public Double value() {
                return motor.getPower();
            }
        });
        telemetry.addData("Intake Position", new Func<Integer>() {
            @Override
            public Integer value() {
                return motor.getCurrentPosition();
            }
        });
        telemetry.addData("Intake Target", new Func<Integer>() {
            @Override
            public Integer value() {
                return motor.getTargetPosition();
            }
        });
    }

    public void start() {
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad1.left_trigger > 0.5 || gamepad2.x)
            // Out
            motor.setPower(-1);
        else if (gamepad1.right_trigger > 0.5 || gamepad2.a)
            // In
            motor.setPower(1);
        else
            motor.setPower(0);
    }

    public void stop() {
    }
}
