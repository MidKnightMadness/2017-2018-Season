package org.firstinspires.ftc.teamcode.MainBot.teleop.GlyphAssembly;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

import static org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator.State.curCol;
import static org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator.State.homeward;
import static org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator.State.justChanged;
import static org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator.State.time;


public class GlyphAssemblyController {

    private static double SERVO_MAX = 1;
    private Telemetry telemetry;
    private DcMotor elev;
    private DcMotor grab;
    private int curLvl;
    private double grabPos;
    private int elevatorTarget;
    private int futureTarget;
    private double futureServo;
    private int timeToNext;
    private boolean dPressed = false;
    private boolean uPressed = false;
    private boolean manual = false;

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        time = new ElapsedTime();
        elev = hardwareMap.dcMotor.get(CrossCommunicator.Glyph.ELEV);
        elev.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elev.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elev.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grab = hardwareMap.dcMotor.get(CrossCommunicator.Glyph.GRAB);
        grab.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grab.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grabPos = 1;
        futureServo = -1;
        elevatorTarget = -1;
        futureTarget = -1;
        timeToNext = 0;
        curLvl = 0;
    }

    public void start() {
        elev.setPower(1);
    }

    private int elevPos() {
        return (elev.getCurrentPosition());
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        boolean up = gamepad1.right_bumper || gamepad2.right_bumper;
        boolean down = gamepad1.left_bumper || gamepad2.left_bumper;
        boolean override = gamepad1.x || gamepad2.x;
        boolean open = gamepad1.right_trigger > 0 || gamepad2.right_trigger > 0;
        boolean close = gamepad1.left_trigger > 0 || gamepad2.left_trigger > 0;

        telemetry.addData("Elevator", elevPos());
        if (up) {
            elevatorTarget = 6600;
            manual = true;
        } else if (down) {
            elevatorTarget = 0;
            manual = true;
        } else if (manual){
            elevatorTarget = elevPos();
        }

        if (override) {
            elev.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elev.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (time.milliseconds() > timeToNext) {
            if (futureTarget == -1 && futureServo != -1) {
                grabPos = futureServo;
                futureServo = -1;
            } else if (futureTarget != -1 && futureServo == -1) {
                elevatorTarget = futureTarget;
                futureTarget = -1;
            }
        }

        if (open) {
            if (grabPos < 0.1) {
                grabPos = 0;
            } else {
                grabPos -= 0.1;
            }
        } else if (close){
            if (grabPos > SERVO_MAX - 0.1) {
                grabPos = SERVO_MAX;
            } else {
                grabPos += 0.1;
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


        if ((gamepad1.dpad_up || gamepad2.dpad_up) && !uPressed) {
            curLvl++;
            curCol = (int)Math.floor(curLvl / 4);
            update();
            uPressed = true;
        } else if (!(gamepad1.dpad_up || gamepad2.dpad_up)){
            uPressed = false;
        }

        if ((gamepad1.dpad_down || gamepad2.dpad_down) && !dPressed) {
            curLvl--;
            curCol = (int)Math.floor(curLvl / 4);
            update();
            dPressed = true;
        } else if (!(gamepad1.dpad_down || gamepad2.dpad_down)){
            dPressed = false;
        }


        telemetry.addData("Servo", grabPos);
        telemetry.addData("Buttons", gamepad1.right_bumper || gamepad1.left_bumper);
        telemetry.addData("Elev", elevPos());
        telemetry.addData("Elevator Target", elevatorTarget);
        telemetry.addData("Future Target", futureTarget);
        telemetry.addData("Time Remaining", time.milliseconds() - timeToNext);


        elev.setPower(Math.min(Math.max(((elevatorTarget)- elev.getCurrentPosition())/300d, -1), 1));
        grab.setPower(Math.min(Math.max(((grabPos*280)-grab.getCurrentPosition())/500d, -1), 1));

        telemetry.addData("Speed Grabber: ", ((grabPos*280)-grab.getCurrentPosition())/500d);
        telemetry.addData("Speed Elevator: ", ((elevatorTarget)- elev.getCurrentPosition())/300d);
    }

    public void update() {
        manual = false;
        elevatorTarget = ((curLvl % 4) * 1000) + 500;
    }

    public void grab() {
        grabLvl(curLvl % 4);
        curLvl++;
        curCol = (int)Math.floor(curLvl / 4);
    }

    public void grabLvl(int level) {
        manual = false;
        grabPos = 0;
        timeToNext = (int)time.milliseconds() + 1000;
        futureTarget = (level * 1000) + 500;
        futureServo = -1;
    }

    public void release() {
        manual = false;
        grabPos = SERVO_MAX;
        timeToNext = (int)time.milliseconds() + 1500;
        futureTarget = 0;
        futureServo = -1;
    }

    public void stop() {

    }
}
