package org.firstinspires.ftc.teamcode.MainBot.teleop.PinballAssembly;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

public class PinballController {

    private enum State {
        HOME_FIRING,
        HOME_FIRING_WAITING,
        HOMING,
        WAITING,
        FIRING,
        WAITING_FOR_FIRE,
        RETRACTING,
        RETRACTING_PHASE_2,
        EXTENDING
    }

    private static final double FLAG_SERVO_POSITION_DOWN = 0;
    private static final double FLAG_SERVO_POSITION_UP = 1;
    private static final double CLAMP_SERVO_POSITION_OPEN = 0.5;
    private static final double CLAMP_SERVO_POSITION_CLOSED = 0;
    private static final int MOTOR_POSITION_SLACKED = -4342;
    private DcMotor motor;
    private Servo clampServo;
    private TouchSensor touchSensor;
    private Servo flagServo;
    private State state = State.HOMING;
    private long waitUntil = -1;
    private int targetPos = 0;

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        motor = hardwareMap.dcMotor.get(CrossCommunicator.Pinball.MOTOR);
        motor.resetDeviceConfigurationForOpMode();
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Pinball Power", new Func<Double>() {
            @Override
            public Double value() {
                return motor.getPower();
            }
        });
        telemetry.addData("Pinball Position", new Func<Integer>() {
            @Override
            public Integer value() {
                return motor.getCurrentPosition();
            }
        });
        telemetry.addData("Pinball Target", new Func<Integer>() {
            @Override
            public Integer value() {
                return motor.getTargetPosition();
            }
        });

        clampServo = hardwareMap.servo.get(CrossCommunicator.Pinball.SERVO_CLAMP);
        touchSensor = hardwareMap.touchSensor.get(CrossCommunicator.Pinball.SENSOR_TOUCH);
        flagServo = hardwareMap.servo.get(CrossCommunicator.Pinball.SERVO_FLAG);
        flagServo.setPosition(FLAG_SERVO_POSITION_DOWN);

        telemetry.addData("Ball Launcher", new Func<String>() {
            @Override
            public String value() {
                return (state == PinballController.State.WAITING ? "Ready" : "Not Ready") +
                        "(" + state.toString() + ")";
            }
        });
    }

    public void start() {
        state = State.HOME_FIRING;
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
            switch (state) {
                case HOME_FIRING:
                    clampServo.setPosition(CLAMP_SERVO_POSITION_OPEN);
                    waitUntil = System.currentTimeMillis() + 250;
                    state = State.HOME_FIRING_WAITING;
                    break;
                case HOME_FIRING_WAITING:
                    if(System.currentTimeMillis() > waitUntil){
                        waitUntil = -1;
                        state = State.HOMING;
                        clampServo.setPosition(CLAMP_SERVO_POSITION_CLOSED);
                    }
                    break;
                case HOMING:
                    if (!touchSensor.isPressed()) {
                        motor.setPower(1);
                        clampServo.setPosition(CLAMP_SERVO_POSITION_CLOSED);
                    } else {
                        motor.setPower(0);
                        DcMotor.RunMode lastMode = motor.getMode();
                        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        motor.setMode(lastMode);
                        state = State.RETRACTING;
                    }
                    break;
                case FIRING:
                    if (!touchSensor.isPressed()) {
                        clampServo.setPosition(CLAMP_SERVO_POSITION_CLOSED);
                        state = State.WAITING_FOR_FIRE;
                        waitUntil = System.currentTimeMillis() + 250;
                    } else {
                        clampServo.setPosition(CLAMP_SERVO_POSITION_OPEN);
                    }
                    break;
                case WAITING_FOR_FIRE:
                    if (System.currentTimeMillis() > waitUntil) {
                        state = State.RETRACTING;
                    }
                    break;
                case RETRACTING:
                    if (!touchSensor.isPressed()) {
                        motor.setPower(1);
                    } else {
                        motor.setPower(0);
                        state = State.RETRACTING_PHASE_2;
                        targetPos = motor.getCurrentPosition() + 200;
                    }
                    break;
                case RETRACTING_PHASE_2:
                    if (motor.getCurrentPosition() >= targetPos) {
                        motor.setPower(0);
                        state = State.EXTENDING;
                    } else {
                        motor.setPower(1);
                    }
                    break;
                case EXTENDING:
                    if (motor.getCurrentPosition() < MOTOR_POSITION_SLACKED) {
                        if (touchSensor.isPressed())
                            state = State.WAITING;
                        else
                            state = State.RETRACTING;
                        motor.setPower(0);
                    } else {
                        motor.setPower(-1);
                    }
                    break;
            }

        if (gamepad1.x || (gamepad2.back || gamepad2.left_bumper || gamepad2.right_bumper)) {
            if (state == State.WAITING) {
                state = State.FIRING;
            }
        }

        if (gamepad2.left_trigger > 0.5 && gamepad2.right_trigger > 0.5) {
            state = State.HOME_FIRING;
        }

        flagServo.setPosition(state == PinballController.State.WAITING ? FLAG_SERVO_POSITION_UP : FLAG_SERVO_POSITION_DOWN);

    }

    public void stop() {
    }
}
