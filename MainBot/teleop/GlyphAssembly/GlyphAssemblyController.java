package org.firstinspires.ftc.teamcode.MainBot.teleop.GlyphAssembly;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

import static org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator.State.justChanged;
import static org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator.State.time;
import static org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator.State.yDecreased;
import static org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator.State.yIncreased;
import static org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator.State.yState;

public class GlyphAssemblyController {

    private static final int UPPER = 0;
    private static final int LOWER = 1;
    private static final int CLOSED = 0;
    private static final int OPEN = 1;
    private static final int HEIGHT_TO_GRAB_SECOND_GLYPH = 1;
    private static final int HEIGHT_AFTER_GRABBING_SECOND_GLYPH = 1;

    private Telemetry telemetry;
    //The DcMotor controlling the elevator vertically
    private DcMotor elev;
    //The DcMotor controlling the main grabber
    private DcMotor grabber[] = new DcMotor[2];
    //The current level (0, 1, 2, or 3) that the robot is placing a glyph at (the current row of the cryptobox)
    private int curLvl;
    //The percentage that the grabber and upperServo are closed: 1 is fully closed
    private double percentageClosed[] = {0, 0};
    //The target position of the elevator
    private int elevatorTargetPos;

    //Future Values: used in the delay after pressing the y-button to give the driver time to get away from the glyphs
    //The future target position of the elevator
    private int futureElevTargetPos;
    //The future percentage to close the grabber and upperServo
    private double futurePercentageClosed[] = {-1, -1};
    //The time at which the future values will come into effect
    private int timeToUpdate;

    //The last known state of the up and down d-pad
    private boolean uPressed = false;
    private boolean dPressed = false;

    //Boolean value representing the manual elevator override mode (allows the elevator to pass the minimum and maximum values)
    private boolean manual = false;

    //Boolean value representing if the main grabber is fully closed on an object
    private boolean isFullyClosed[] = {false, false};
    //The last difference in movement of the grabber: used in determining if the grabber is fully closed
    private int lastdiff[] = {0, 0};

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;

        //Initialize the time variable for later
        time = new ElapsedTime();

        //Initialize the elevator motor: reset encoder, RUN_USING_ENCODER, and brake on zero power
        elev = hardwareMap.dcMotor.get(CrossCommunicator.Glyph.ELEV);
        elev.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elev.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elev.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabber[0] = hardwareMap.dcMotor.get(CrossCommunicator.Glyph.GRAB_UPPER);
        grabber[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabber[1] = hardwareMap.dcMotor.get(CrossCommunicator.Glyph.GRAB_LOWER);
        grabber[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorTargetPos = -1;
        futureElevTargetPos = -1;
        timeToUpdate = 0;
        //curLvl = 0;
    }

    public void start() {
        //init grabber if necessary

        grabber[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        grabber[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    private int elevPos() {
        return (elev.getCurrentPosition());
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        boolean up = gamepad1.right_bumper  || gamepad2.left_stick_y < -0.1;
        boolean down = gamepad1.left_bumper || gamepad2.left_stick_y > 0.1;
        boolean override = gamepad1.x || gamepad2.x;
        boolean grab[][] = {
                {
                        gamepad2.left_bumper, //upper closed
                        gamepad2.right_bumper //upper open
                }, {
                        gamepad1.left_trigger > 0 || gamepad2.left_trigger > 0, //lower closed
                        gamepad1.right_trigger > 0 || gamepad2.right_trigger > 0 //lower open
                }};

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

        /*if (override) {
            elev.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elev.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }*/

        if (time.milliseconds() > timeToUpdate) {
            if (futurePercentageClosed[0] != -1) {
                percentageClosed[0] = futurePercentageClosed[0];
                futurePercentageClosed[0] = -1;
            }
            if (futurePercentageClosed[1] != -1) {
                percentageClosed[1] = futurePercentageClosed[1];
                futurePercentageClosed[1] = -1;
            }
            if (futureElevTargetPos != -1) {
                elevatorTargetPos = futureElevTargetPos;
                futureElevTargetPos = -1;
            }
        }

        for (int i = 0; i < 2; i++) {
            if (grab[i][CLOSED]) {
                if (percentageClosed[i] > 0.9) {
                    percentageClosed[i] = 1;
                } else {
                    percentageClosed[i] += 0.1;
                }
            } else if (grab[i][OPEN]) {
                if (percentageClosed[i] < 0.1) {
                    percentageClosed[i] = 0;
                } else {
                    percentageClosed[i] -= 0.1;
                }
            }

            if (Math.abs(lastdiff[i] - (percentageClosed[i] * 800 - grabber[i].getCurrentPosition())) < 10) {
                isFullyClosed[i] = true;
            } else {
                isFullyClosed[i] = false;
            }
            lastdiff[i] = Math.abs((int) (percentageClosed[i] * 800 - grabber[i].getCurrentPosition()));
        }

        if (yIncreased) {
            if (yState == 0 || yState == 1) {
                grab(yState);
            } else {
                releaseBoth();
            }
            justChanged = false;
        } else if (yDecreased) {
            if (yState == 2) {
                release(0);
            } else if (yState == 0){
                release(1);
            } else {
                releaseBoth();
            }
        }

        telemetry.addData("Grabber", percentageClosed);
        telemetry.addData("Buttons", gamepad1.right_bumper || gamepad1.left_bumper);
        telemetry.addData("Elev", elevPos());
        telemetry.addData("Elevator Target", elevatorTargetPos);
        telemetry.addData("Future Target", futureElevTargetPos);
        telemetry.addData("Time Remaining", time.milliseconds() - timeToUpdate);

        elev.setTargetPosition((elevatorTargetPos)- elev.getCurrentPosition());
        grabber[UPPER].setPower(Math.min(Math.max(((percentageClosed[UPPER] *800)- grabber[UPPER].getCurrentPosition())/300d,
                isFullyClosed[UPPER] ? -0.03 : -0.5),
                isFullyClosed[UPPER] ? 0.03 : 0.5));
        grabber[LOWER].setPower(Math.min(Math.max(((percentageClosed[LOWER] *800)- grabber[LOWER].getCurrentPosition())/300d,
                isFullyClosed[LOWER] ? -0.03 : -0.5),
                isFullyClosed[LOWER] ? 0.03 : 0.5));

        telemetry.addData("Speed Upper Grabber: ", grabber[0].getPower());
        telemetry.addData("Speed Lower Grabber: ", grabber[1].getPower());
        telemetry.addData("Position Elevator: ", ((elevatorTargetPos)- elev.getCurrentPosition()));
        telemetry.addData("Fully Closed Upper", isFullyClosed[0]);
        telemetry.addData("Fully Closed Lower", isFullyClosed[1]);
    }

    public void releaseAll() {

    }

    public void grab(int arm) {
        manual = false;
        percentageClosed[arm] = 1;
        timeToUpdate = (int)time.milliseconds() + 1000;
        futureElevTargetPos = arm == UPPER ? HEIGHT_TO_GRAB_SECOND_GLYPH : HEIGHT_AFTER_GRABBING_SECOND_GLYPH;
        futurePercentageClosed[0] = -1;
        futurePercentageClosed[1] = -1;
    }

    public void release(int arm) {
        manual = false;
        percentageClosed[arm] = 0.4;
        timeToUpdate = (int) time.milliseconds() + 1500;
        futureElevTargetPos = arm == LOWER ? HEIGHT_TO_GRAB_SECOND_GLYPH : 0;
        futurePercentageClosed[arm] = 0;
        futurePercentageClosed[arm == 0 ? 1 : 0] = -1;
    }

    public void releaseBoth() {
        release(LOWER);
        release(UPPER);
        futurePercentageClosed[LOWER] = 0;
    }

    public void stop() {

    }
}
