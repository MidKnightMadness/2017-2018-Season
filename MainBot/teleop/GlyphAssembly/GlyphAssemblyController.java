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
import static org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator.State.time;

public class GlyphAssemblyController {

    private Telemetry telemetry;
    //The DcMotor controlling the elevator vertically
    private DcMotor elev;
    //The DcMotor controlling the main grabber
    private DcMotor grabber;
    //The servo controlling the upper contact point to hold onto two glyphs at a time
    private Servo upperServo;
    //The current level (0, 1, 2, or 3) that the robot is placing a glyph at (the current row of the cryptobox)
    private int curLvl;
    //The percentage that the grabber and upperServo are closed: 1 is fully closed
    private double percentageClosed;
    //The target position of the elevator TODO: use RUN_TO_POSITION mode?
    private int elevatorTargetPos;

    //Future Values: used in the delay after pressing the y-button to give the driver time to get away from the glyphs
    //The future target position of the elevator
    private int futureElevTargetPos;
    //The future percentage to close the grabber and upperServo
    private double futurePercentageClosed;
    //The time at which the future values will come into effect
    private int timeToUpdate;

    //The last known state of the up and down d-pad
    private boolean uPressed = false;
    private boolean dPressed = false;

    //Boolean value representing the manual elevator override mode (allows the elevator to pass the minimum and maximum values)
    private boolean manual = false;

    //Boolean value representing if the main grabber is fully closed on an object
    private boolean isFullyClosed = false;
    //The last difference in movement of the grabber: used in determining if the grabber is fully closed
    private int lastdiff = 0;

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;

        //Initialize the time variable for later
        time = new ElapsedTime();

        //Initialize the elevator motor: reset encoder, RUN_USING_ENCODER, and brake on zero power
        elev = hardwareMap.dcMotor.get(CrossCommunicator.Glyph.ELEV);
        elev.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elev.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elev.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabber = hardwareMap.dcMotor.get(CrossCommunicator.Glyph.GRAB);
        grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        upperServo = hardwareMap.servo.get(CrossCommunicator.Glyph.UPPER_SERVO);
        upperServo.scaleRange(0, 1);
        percentageClosed = 0;
        futurePercentageClosed = -1;
        elevatorTargetPos = -1;
        futureElevTargetPos = -1;
        timeToUpdate = 0;
        curLvl = 0;
    }

    public void start() {
        upperServo.setPosition(0);
        grabber.setTargetPosition(200);
        grabber.setPower(0.2);
        while (grabber.isBusy()) {}
        grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private int elevPos() {
        return (elev.getCurrentPosition());
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        boolean up = gamepad1.right_bumper || gamepad2.right_trigger > 0 || gamepad2.left_stick_y > 0.1;
        boolean down = gamepad1.left_bumper || gamepad2.left_trigger > 0 || gamepad2.left_stick_y < -0.1;
        boolean override = gamepad1.x || gamepad2.x;
        boolean open = gamepad1.right_trigger > 0 || gamepad2.right_bumper;
        boolean close = gamepad1.left_trigger > 0 || gamepad2.left_bumper;

        telemetry.addData("Elevator", elevPos());
        if (up) {
            elevatorTargetPos = 6600;
            manual = true;
        } else if (down) {
            elevatorTargetPos = 0;
            manual = true;
        } else if (manual){
            elevatorTargetPos = elevPos();
            manual = false;
        }

        if (override) {
            elev.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elev.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (time.milliseconds() > timeToUpdate) {
            if (futurePercentageClosed != -1) {
                percentageClosed = futurePercentageClosed;
                futurePercentageClosed = -1;
            }
            if (futureElevTargetPos != -1) {
                elevatorTargetPos = futureElevTargetPos;
                futureElevTargetPos = -1;
            }
        }

        if (open) {
            if (percentageClosed < 0.1) {
                percentageClosed = 0;
            } else {
                percentageClosed -= 0.1;
            }
        } else if (close){
            if (percentageClosed > 0.9) {
                percentageClosed = 1;
            } else {
                percentageClosed += 0.1;
            }
        }

        if (Math.abs(lastdiff - (percentageClosed * 800 - grabber.getCurrentPosition())) < 10) {
            isFullyClosed = true;
        } else {
            isFullyClosed = false;
        }
        lastdiff = Math.abs((int)(percentageClosed * 800 - grabber.getCurrentPosition()));

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


        telemetry.addData("Grabber", percentageClosed);
        telemetry.addData("Buttons", gamepad1.right_bumper || gamepad1.left_bumper);
        telemetry.addData("Elev", elevPos());
        telemetry.addData("Elevator Target", elevatorTargetPos);
        telemetry.addData("Future Target", futureElevTargetPos);
        telemetry.addData("Time Remaining", time.milliseconds() - timeToUpdate);

        elev.setPower(Math.min(Math.max(((elevatorTargetPos)- elev.getCurrentPosition())/300d, -1), 1));
        grabber.setPower(Math.min(Math.max(((percentageClosed *800)- grabber.getCurrentPosition())/300d,
                isFullyClosed ? -0.03 : -0.5),
                isFullyClosed ? 0.03 : 0.5));
        upperServo.setPosition(percentageClosed);

        telemetry.addData("Speed Grabber: ", grabber.getPower());
        telemetry.addData("Speed Elevator: ", ((elevatorTargetPos)- elev.getCurrentPosition())/300d);
        telemetry.addData("Fully Closed", isFullyClosed);
    }

    public void update() {
        manual = false;
        elevatorTargetPos = ((curLvl % 4) * 1000) + 500;
    }

    public void grab() {
        grabLvl(curLvl % 4);
        curLvl++;
        curCol = (int)Math.floor(curLvl / 4);
    }

    public void grabLvl(int level) {
        manual = false;
        percentageClosed = 1;
        timeToUpdate = (int)time.milliseconds() + 1000;
        futureElevTargetPos = (level * 1000) + 500;
        futurePercentageClosed = -1;
    }

    public void release() {
        manual = false;
        percentageClosed = 0.4;
        timeToUpdate = (int)time.milliseconds() + 1500;
        futureElevTargetPos = 0;
        futurePercentageClosed = 0;
    }

    public void stop() {

    }
}
