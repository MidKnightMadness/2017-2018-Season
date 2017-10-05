package org.firstinspires.ftc.teamcode.MiniBot.teleop.CircularDriveAssembly;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MiniBot.teleop.CrossCommunicator;

public class CircularDriveController {
    //private DcMotor left;
    private DcMotor right;

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        right = hardwareMap.dcMotor.get(CrossCommunicator.Drive.MOTOR_RIGHT);
        right.setDirection(DcMotor.Direction.FORWARD);

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
        if (gamepad1.right_bumper) {
            right.setDirection(DcMotor.Direction.FORWARD);
            right.setPower(1);
        } else if (gamepad1.left_bumper) {
            right.setDirection(DcMotor.Direction.REVERSE);
            right.setPower(1);
        } else {
            right.setPower(0);
        }
    }

    public void stop() {
    }
}
