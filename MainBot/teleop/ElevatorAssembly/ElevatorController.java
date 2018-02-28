package org.firstinspires.ftc.teamcode.MainBot.teleop.ElevatorAssembly;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

public class ElevatorController {

    private static final double SERVO_POSITION_CLOSED = 0;
    private static final double SERVO_POSITION_OPEN = 1;
    private DcMotor motor;
    private Servo servo;

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        motor = hardwareMap.dcMotor.get(CrossCommunicator.Elevator.MOTOR);
        motor.resetDeviceConfigurationForOpMode();
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Elevator Power", new Func<Double>() {
            @Override
            public Double value() {
                return motor.getPower();
            }
        });
        telemetry.addData("Elevator Position", new Func<Integer>() {
            @Override
            public Integer value() {
                return motor.getCurrentPosition();
            }
        });
        telemetry.addData("Elevator Target", new Func<Integer>() {
            @Override
            public Integer value() {
                return motor.getTargetPosition();
            }
        });

        servo = hardwareMap.servo.get(CrossCommunicator.Elevator.SERVO);
        servo.setDirection(Servo.Direction.REVERSE);
        servo.setPosition(SERVO_POSITION_CLOSED);
    }

    public void start() {
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad1.right_bumper || gamepad2.dpad_up) {
            servo.setPosition(SERVO_POSITION_OPEN);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(1);
        } else if (gamepad1.left_bumper || gamepad2.dpad_down) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(-1);
        } else {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setTargetPosition(motor.getCurrentPosition());
        }

        if (gamepad1.b || gamepad2.dpad_right) {
            servo.setPosition(SERVO_POSITION_OPEN);
        } else if (gamepad1.y || gamepad2.dpad_left) {
            servo.setPosition(SERVO_POSITION_CLOSED);
        }
        if (Math.abs(motor.getCurrentPosition()) > 20000) {
            CrossCommunicator.Elevator.setIsExtended(true);
        } else {
            CrossCommunicator.Elevator.setIsExtended(false);
        }
    }

    public void stop() {
    }
}
