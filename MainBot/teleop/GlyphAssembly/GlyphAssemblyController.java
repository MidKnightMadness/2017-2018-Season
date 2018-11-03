package org.firstinspires.ftc.teamcode.MainBot.teleop.GlyphAssembly;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;

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
    private static final int HEIGHT_TO_GRAB_SECOND_GLYPH = 2200;
    private static final int HEIGHT_AFTER_GRABBING_SECOND_GLYPH = 5600;
    private static final double K_DISTANCE = 0.0005;
    private static final double K_VELOCITY = 0.0002;
    private static final double K_TOTAL[] = {2, 1.5};
    private static final int DISTANCE_FROM_HARD_STOP[] = {140, 100};

    private boolean bPressed;
    private boolean resettingArms;

    private Servo vsd;
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
    private double timeToUpdate;

    //The last known state of the up and down d-pad
    private boolean uPressed = false;
    private boolean dPressed = false;

    //Boolean value representing the manual elevator override mode (allows the elevator to pass the minimum and maximum values)
    private boolean manual = false;

    //Boolean value representing if the main grabber is fully closed on an object
    private boolean isFullyClosed[] = {false, false};
    //The last difference in movement of the grabber: used in determining if the grabber is fully closed
    private int lastpos[] = {0, 0};


    ElapsedTime elapsedTime = new ElapsedTime();
    FileOutputStream fileOutputStream;
    OutputStreamWriter outputStreamWriter;
    int lastPosition[] = {0, 0};
    double lastTime[] = {0, 0};
    double velocity[] = {0, 0};
    double thisTime[] = {0, 0};
    double thisEnc[] = {0, 0};
    //double velocityArray[][] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
    int i = 0;



    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;

        //Initialize the time variable for later
        time = new ElapsedTime();

        //Initialize the elevator motor: reset encoder, RUN_USING_ENCODER, and brake on zero power
        elev = hardwareMap.dcMotor.get(CrossCommunicator.Glyph.ELEV);
        elev.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elev.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elev.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabber[0] = hardwareMap.dcMotor.get(CrossCommunicator.Glyph.GRAB_UPPER);
        grabber[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabber[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabber[1] = hardwareMap.dcMotor.get(CrossCommunicator.Glyph.GRAB_LOWER);
        grabber[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabber[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabber[1].setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorTargetPos = -1;
        futureElevTargetPos = -1;
        timeToUpdate = -1;

        vsd = hardwareMap.servo.get(CrossCommunicator.Glyph.VSD);

        try {
            int j = 0;
            while (new File("/storage/self/primary/Data/", j + "Data.txt").exists()) {
                j++;
            }
            File file = new File("/storage/self/primary/Data/", j + "Data.txt");
            fileOutputStream = new FileOutputStream(file);
            outputStreamWriter = new OutputStreamWriter(fileOutputStream);
        } catch (Exception e) {
            telemetry.addData("Error", e);
            telemetry.update();
        }
    }

    public void start() {

        vsd.setPosition(0);
        //init grabber if necessary

        grabber[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        grabber[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resettingArms = true;
        percentageClosed[0] = 0;
        percentageClosed[1] = 0;
        grabber[UPPER].setPower(-0.5);
        grabber[LOWER].setPower(-0.5);


        elapsedTime.reset();
    }

    private int elevPos() {
        return (elev.getCurrentPosition());
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        updateVelocity();
        boolean up = gamepad1.right_bumper  || gamepad2.left_stick_y < -0.1;
        boolean down = gamepad1.left_bumper || gamepad2.left_stick_y > 0.1;
        boolean override = gamepad2.x;
        boolean grab[][] = {
                {
                        gamepad2.left_bumper,
                        gamepad2.right_bumper
                }, {
                        gamepad1.right_trigger > 0 || gamepad2.left_trigger > 0, //lower closed (1, 0)
                        gamepad1.left_trigger > 0 || gamepad2.right_trigger > 0 //lower open
                }
        };

        if ((gamepad1.b || gamepad2.b )&& !bPressed) {
            bPressed = true;
            resettingArms = true;
            percentageClosed[0] = 0;
            percentageClosed[1] = 0;
            grabber[UPPER].setPower(-0.5);
            grabber[LOWER].setPower(-0.5);
        } else if (!(gamepad1.b|| gamepad2.b)){
            bPressed = false;
        }

        telemetry.addData("Elevator", elevPos());
        if (up) {
            elevatorTargetPos = HEIGHT_AFTER_GRABBING_SECOND_GLYPH;
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
            elev.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }*/

        if (time.seconds() > timeToUpdate && timeToUpdate != -1) {
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
            timeToUpdate = -1;
        }

        if (!resettingArms) {
            for (int i = 0; i < 2; i++) {
                if (grab[i][CLOSED]) {
                    if (percentageClosed[i] > 0.9) {
                        percentageClosed[i] = 1;
                    } else {
                        percentageClosed[i] += 0.15;
                    }
                } else if (grab[i][OPEN]) {
                    if (percentageClosed[i] < 0.1) {
                        percentageClosed[i] = 0;
                    } else {
                        percentageClosed[i] -= 0.15;
                    }
                }

                isFullyClosed[i] = (Math.abs(lastpos[i] - grabber[i].getCurrentPosition()) < 5);
                lastpos[i] = grabber[i].getCurrentPosition();
            }
        } else {
            if (!bPressed && time.seconds() > 2) {
                grabber[UPPER].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                grabber[UPPER].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                grabber[LOWER].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                grabber[LOWER].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                resettingArms = false;
            }
        }
        if (justChanged) {
            if (yIncreased) {
                if (yState == 0 || yState == 1) {
                    grab(yState);
                } else if (yState == 2){
                    releaseBoth();
                } else {
                    finishReleaseBoth();
                }
            } else if (yDecreased) {
                if (yState == 3) {
                    release(0);
                } else if (yState == 0) {
                    release(1);
                } else if (yState == 2){
                    releaseBoth();
                    finishReleaseBoth();
                }
            }
            justChanged = false;
        }

        telemetry.addData("FRONT_LEFT", up);
        telemetry.addData("BACK_RIGHT", down);
        telemetry.addData("open_Lower", grab[LOWER][OPEN]);
        telemetry.addData("close_Lower", grab[LOWER][CLOSED]);
        telemetry.addData("open_Upper", grab[UPPER][OPEN]);
        telemetry.addData("close_Upper", grab[UPPER][CLOSED]);
        telemetry.addData("Percentage Closed Upper", percentageClosed[UPPER]);
        telemetry.addData("Percentage Closed Lower", percentageClosed[LOWER]);
        telemetry.addData("YDecreased", yDecreased);
        telemetry.addData("YState", yState);
        telemetry.addData("Just Changed", justChanged);

        telemetry.addData("Buttons", gamepad1.right_bumper || gamepad1.left_bumper);
        telemetry.addData("Elev", elevPos());
        telemetry.addData("Elevator Target", elevatorTargetPos);
        telemetry.addData("Future Target", futureElevTargetPos);
        telemetry.addData("Time To Update", timeToUpdate);
        telemetry.addData("Current Time", time.seconds());

        elev.setTargetPosition(elevatorTargetPos);
        elev.setPower(1);
        //elev.setPower(Math.min(Math.max(elevatorTargetPos - elev.getCurrentPosition()/10d, -1), 1));
        if (!resettingArms) {
            grabber[UPPER].setPower(Math.min(Math.max(
                    K_TOTAL[UPPER] * (K_DISTANCE * ((DISTANCE_FROM_HARD_STOP[UPPER] + percentageClosed[UPPER] * 800) - grabber[UPPER].getCurrentPosition()) - (K_VELOCITY * velocity[UPPER]))
                    , -0.6), 0.6));
            telemetry.addData("Upper Power Unrestricted", K_TOTAL[UPPER] * (K_DISTANCE * ((DISTANCE_FROM_HARD_STOP[UPPER] + percentageClosed[UPPER] * 800) - grabber[UPPER].getCurrentPosition()) - (K_VELOCITY * velocity[UPPER])));
            grabber[LOWER].setPower(Math.min(Math.max(
                    K_TOTAL[LOWER] * (K_DISTANCE * ((DISTANCE_FROM_HARD_STOP[LOWER] + percentageClosed[LOWER] * 800) - grabber[LOWER].getCurrentPosition()) - (K_VELOCITY * velocity[LOWER]))
                    , -0.6), 0.6));
        }
        telemetry.addData("Current Upper Grabber Position", grabber[UPPER].getCurrentPosition());
        telemetry.addData("Distance Upper", ((DISTANCE_FROM_HARD_STOP[UPPER] + percentageClosed[UPPER] * 800) - grabber[UPPER].getCurrentPosition()));
        telemetry.addData("Speed Upper Grabber: ", grabber[0].getPower());
        telemetry.addData("Speed Lower Grabber: ", grabber[1].getPower());
        telemetry.addData("Position Elevator: ", elevPos());
        telemetry.addData("Fully Closed Upper", isFullyClosed[0]);
        telemetry.addData("Fully Closed Lower", isFullyClosed[1]);
    }

    public void releaseAll() {

    }

    public void grab(int arm) {
        vsd.setPosition((arm + 1) / 2d);
        manual = false;
        percentageClosed[arm] = 1;
        timeToUpdate = time.seconds() + 1d;
        futureElevTargetPos = arm == UPPER ? HEIGHT_TO_GRAB_SECOND_GLYPH : HEIGHT_AFTER_GRABBING_SECOND_GLYPH;
        futurePercentageClosed[0] = -1;
        futurePercentageClosed[1] = -1;
    }

    public void release(int arm) {
        vsd.setPosition(0);
        manual = false;
        percentageClosed[arm] = 0.2;
        timeToUpdate = time.seconds() + 1.5d;
        futureElevTargetPos = arm == LOWER ? HEIGHT_TO_GRAB_SECOND_GLYPH : 0;
        futurePercentageClosed[arm] = 0;
        futurePercentageClosed[arm == 0 ? 1 : 0] = -1;
    }

    public void finishReleaseBoth() {
        percentageClosed[0] = 0;
        percentageClosed[1] = 0;
        elevatorTargetPos = 0;
    }

    public void releaseBoth() {
        release(LOWER);
        release(UPPER);
        futurePercentageClosed[0] = -1;
        futurePercentageClosed[1] = -1;
        futureElevTargetPos = -1;
    }

    public void stop() {
        try {
            outputStreamWriter.flush();
            outputStreamWriter.close();
            fileOutputStream.flush();
            fileOutputStream.close();
        } catch (Exception e) {

        }
    }




    private void updateVelocity() {
        //i = i + 1 < 10 ? i + 1 : 0;
        for (int mi = 0; mi < 2; mi++) {

            thisEnc[mi] = lastPosition[mi] - grabber[mi].getCurrentPosition();
            lastPosition[mi] = grabber[mi].getCurrentPosition();
            thisTime[mi] = lastTime[mi] - elapsedTime.seconds();
            lastTime[mi] = elapsedTime.seconds();

            //velocityArray[mi][i] = thisEnc[mi] / thisTime[mi];
            /*velocity[mi] = 0;
            for (int j = 0; j < 10; j++) {
                velocity[mi] += velocityArray[mi][j];
            }*/
            velocity[mi] = 0.7 * velocity[mi] + 0.3 * (thisEnc[mi] / thisTime[mi]);

            telemetry.addData("Velocity (enc/sec)", velocity[mi]);
            telemetry.addData("Time This Run", thisTime[mi]);
            telemetry.addData("Enc This Run", thisEnc[mi]);
        }
    }
}
