package org.firstinspires.ftc.teamcode.MainBot.teleop.GlyphAssembly;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

public class GlyphAssemblyController {

    private Telemetry telemetry;
    private DcMotor motor;
    private Servo servo;
    private boolean open;
    private int pos;
    private double servoPos;
    private int elevatorTarget;
    private int futureTarget;
    private int targetTime;
    private ElapsedTime time;

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        time = new ElapsedTime();
        motor = hardwareMap.dcMotor.get(CrossCommunicator.Glyph.MOTOR);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo = hardwareMap.servo.get(CrossCommunicator.Glyph.SERVO);
        servo.setPosition(0.5);
        open = true;
        pos = motor.getCurrentPosition();
        servoPos = 0.5;
        elevatorTarget = -1;
        futureTarget = -1;
        targetTime = 0;
    }

    public void start() {

    }

    private int elevPos() {
        return (motor.getCurrentPosition() - pos);
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {

        if (gamepad1.right_bumper || gamepad1.left_bumper/* || gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < 0.1*/) {
            elevatorTarget = -1;
        }
        telemetry.addData("Elevator", elevPos());
        if (gamepad1.right_bumper || ((elevatorTarget - elevPos()) > 100 && elevatorTarget != -1)) {
            motor.setPower((elevPos() < 6600 || gamepad1.x) ? 1 : 0);
        } else if (gamepad1.left_bumper ||((elevatorTarget - elevPos()) < -100 && elevatorTarget != -1)) {
            motor.setPower((elevPos() > 0 || gamepad1.x) ? -1 : 0);
        } else if (gamepad2.left_stick_y > 0.1) {
            motor.setPower((elevPos() < 6600) ? gamepad2.left_stick_y : 0);
        } else if (gamepad2.left_stick_y < 0.1) {
            motor.setPower((elevPos() > 0) ? gamepad2.left_stick_y : 0);
        } else {
            motor.setPower(0);
        }

        if (gamepad1.x) {
            pos = motor.getCurrentPosition();
        }

        if (time.milliseconds() > targetTime && futureTarget != -1) {
            elevatorTarget = futureTarget;
            futureTarget = -1;
        }

        if (gamepad1.dpad_left || gamepad1.right_trigger > 0) {
            if (servoPos < 0.1) {
                servoPos = 0;
            } else {
                servoPos -= 0.1;
            }
        } else if (gamepad1.dpad_right || gamepad1.left_trigger > 0){
            if (servoPos > 0.45) {
                servoPos = 0.55;
            } else {
                servoPos += 0.1;
            }
        }

        if (gamepad1.dpad_up) {
            grab(2);
        } else if (gamepad1.dpad_down) {
            release();
        }



        telemetry.addData("Servo", servoPos);
        telemetry.addData("Buttons", gamepad1.right_bumper || gamepad1.left_bumper/* || gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < 0.1*/);
        telemetry.addData("Elev", motor.getCurrentPosition());
        telemetry.addData("Elevator Target", elevatorTarget);
        telemetry.addData("Future Target", futureTarget);
        telemetry.addData("Time Remaining", time.milliseconds() - targetTime);
        servo.setPosition(servoPos);


        telemetry.addLine("Servo Position: " + servo.getPosition());
    }


    public void grab(int level) {
        servoPos = 0;
        targetTime = (int)time.milliseconds() + 1000;
        futureTarget = level * 1250;
    }

    public void release() {
        elevatorTarget = elevPos() - 200;
        servoPos = 0.5;
    }

    public void stop() {

    }
}
