package org.firstinspires.ftc.teamcode.MainBot.teleop.GlyphAssembly;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

import static org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator.State.curCol;
import static org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator.State.homeward;
import static org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator.State.justChanged;

public class GlyphAssemblyController {

    private Telemetry telemetry;
    private DcMotor motor;
    private Servo servo;
    private int curLvl;
    private int pos;
    private double servoPos;
    private int elevatorTarget;
    private int futureTarget;
    private double futureServo;
    private int timeToNext;
    private ElapsedTime time;

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        time = new ElapsedTime();
        motor = hardwareMap.dcMotor.get(CrossCommunicator.Glyph.MOTOR);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo = hardwareMap.servo.get(CrossCommunicator.Glyph.SERVO);
        servo.setPosition(1);
        pos = motor.getCurrentPosition();
        servoPos = 1;
        futureServo = -1;
        elevatorTarget = -1;
        futureTarget = -1;
        timeToNext = 0;
        curLvl = 0;
    }

    public void start() {

    }

    private int elevPos() {
        return (motor.getCurrentPosition() - pos);
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {

        if (Math.abs(elevatorTarget - elevPos()) < 50 || gamepad1.right_bumper || gamepad1.left_bumper) {
            elevatorTarget = -1;
        }
        boolean up = gamepad1.right_bumper || (elevatorTarget - elevPos() > 50 && elevatorTarget != -1);
        boolean down = gamepad1.left_bumper || (elevatorTarget - elevPos() < -50 && elevatorTarget != -1);
        boolean override = gamepad1.x;
        boolean open = gamepad1.dpad_left || gamepad1.right_trigger > 0;
        boolean close = gamepad1.dpad_right || gamepad1.left_trigger > 0;

        telemetry.addData("Elevator", elevPos());
        if (up) {
            motor.setPower((elevPos() < 6600 || override) ? 1 : 0);
        } else if (down) {
            motor.setPower((elevPos() > 0 || override) ? -1 : 0);
        } else {
            motor.setPower(0);
        }

        if (override) {
            pos = motor.getCurrentPosition();
        }

        if (time.milliseconds() > timeToNext) {
            if (futureTarget == -1 && futureServo != -1) {
                servoPos = futureServo;
                futureServo = -1;
            } else if (futureTarget != -1 && futureServo == -1) {
                elevatorTarget = futureTarget;
                futureTarget = -1;
            }
        }

        if (open) {
            if (servoPos < 0.1) {
                servoPos = 0;
            } else {
                servoPos -= 0.1;
            }
        } else if (close){
            if (servoPos > 0.9) {
                servoPos = 1;
            } else {
                servoPos += 0.1;
            }
        }


        if (justChanged) {
            if (homeward) {
                grab();
            } else {
                release();
            }
            justChanged = false;
        }

        if (gamepad1.dpad_up) {
            curLvl++;
            curCol = (int)Math.floor(curLvl / 4);
            update();
        } else if (gamepad1.dpad_down) {
            curLvl--;
            curCol = (int)Math.floor(curLvl / 4);
            update();
        }



        telemetry.addData("Servo", servoPos);
        telemetry.addData("Buttons", gamepad1.right_bumper || gamepad1.left_bumper/* || gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < 0.1*/);
        telemetry.addData("Elev", motor.getCurrentPosition());
        telemetry.addData("Elevator Target", elevatorTarget);
        telemetry.addData("Future Target", futureTarget);
        telemetry.addData("Time Remaining", time.milliseconds() - timeToNext);
        servo.setPosition(servoPos);


        telemetry.addLine("Servo Position: " + servo.getPosition());
    }

    public void update() {
        elevatorTarget = ((curLvl % 4) * 1250) + 200;
    }

    public void grab() {
        grabLvl(curLvl % 4);
        curLvl++;
        curCol = (int)Math.floor(curLvl / 4);
    }

    public void grabLvl(int level) {
        servoPos = 0;
        timeToNext = (int)time.milliseconds() + 500;
        futureTarget = (level * 1250) + 200;
        futureServo = -1;
    }

    public void release() {
        elevatorTarget = 0;
        timeToNext = (int)time.milliseconds() + 500;
        futureTarget = -1;
        futureServo = 0.5;
    }

    public void stop() {

    }
}
