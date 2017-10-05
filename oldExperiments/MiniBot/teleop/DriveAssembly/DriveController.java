package org.firstinspires.ftc.teamcode.MiniBot.teleop.DriveAssembly;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MiniBot.teleop.CrossCommunicator;

public class DriveController {
    private DcMotor left;
    private DcMotor right;

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        left = hardwareMap.dcMotor.get(CrossCommunicator.Drive.MOTOR_LEFT);
        left.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData("Left Power", new Func<Double>() {
            @Override
            public Double value() {
                return left.getPower();
            }
        });
        telemetry.addData("Left Position", new Func<Integer>() {
            @Override
            public Integer value() {
                return left.getCurrentPosition();
            }
        });
        telemetry.addData("Left Target", new Func<Integer>() {
            @Override
            public Integer value() {
                return left.getTargetPosition();
            }
        });

        right = hardwareMap.dcMotor.get(CrossCommunicator.Drive.MOTOR_RIGHT);
        right.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Right Power", new Func<Double>() {
            @Override
            public Double value() {
                return right.getPower();
            }
        });
        telemetry.addData("Right Position", new Func<Integer>() {
            @Override
            public Integer value() {
                return right.getCurrentPosition();
            }
        });
        telemetry.addData("Right Target", new Func<Integer>() {
            @Override
            public Integer value() {
                return right.getTargetPosition();
            }
        });
    }

    public void start() {
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        left.setPower(gamepad1.left_stick_y);
        right.setPower(gamepad1.right_stick_y);
    }

    public void stop() {
    }
}
