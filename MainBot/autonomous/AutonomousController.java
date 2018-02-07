package org.firstinspires.ftc.teamcode.MainBot.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;


public class AutonomousController {

    public static int ELEV = 0;
    public static int JEWEL = 1;
    public static int UP = 2;
    public static int DOWN = 3;
    public static int LEFT = 4;
    public static int RIGHT = 5;
    public static int GRAB = 6;

    private VisualController v;
    private static double DRIVE_SPEED = 0.4;
    private static double ROTATE_SPEED = 0.3;



    public DcMotor[] motors = new DcMotor[7];

    public void init(Telemetry telemetry, HardwareMap hardwareMap, VisualController v) {
        this.v = v;
        motors[ELEV] = hardwareMap.dcMotor.get(CrossCommunicator.Glyph.ELEV);
        motors[ELEV].resetDeviceConfigurationForOpMode();
        motors[ELEV].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors[ELEV].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motors[JEWEL] = hardwareMap.dcMotor.get(CrossCommunicator.Jewel.MOTOR);
        motors[JEWEL].resetDeviceConfigurationForOpMode();
        motors[JEWEL].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors[JEWEL].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motors[UP] = hardwareMap.dcMotor.get(CrossCommunicator.Drive.FRONT_LEFT);
        motors[UP].resetDeviceConfigurationForOpMode();
        motors[UP].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors[UP].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motors[DOWN] = hardwareMap.dcMotor.get(CrossCommunicator.Drive.BACK_RIGHT);
        motors[DOWN].resetDeviceConfigurationForOpMode();
        motors[DOWN].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors[DOWN].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motors[LEFT] = hardwareMap.dcMotor.get(CrossCommunicator.Drive.BACK_LEFT);
        motors[LEFT].resetDeviceConfigurationForOpMode();
        motors[LEFT].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors[LEFT].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motors[RIGHT] = hardwareMap.dcMotor.get(CrossCommunicator.Drive.FRONT_RIGHT);
        motors[RIGHT].resetDeviceConfigurationForOpMode();
        motors[RIGHT].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors[RIGHT].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motors[GRAB] = hardwareMap.dcMotor.get(CrossCommunicator.Glyph.GRAB_UPPER);
        motors[GRAB].resetDeviceConfigurationForOpMode();
        motors[GRAB].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[GRAB].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[GRAB].setDirection(DcMotorSimple.Direction.REVERSE);

        reset();
    }

    private void move(int motor, int relTarget, double speed) {
        motors[motor].setTargetPosition(motors[motor].getCurrentPosition() + relTarget);
        motors[motor].setPower(-speed);
    }

    public void close() {
        motors[GRAB].setPower(0.3);
    }

    public void grabStop() {
        motors[GRAB].setPower(0);
    }
    public void open() {
        motors[GRAB].setPower(-0.8);
    }
    public void lift() {
        move(ELEV, 1000, 1);
    }
    public void lower() {
        move(ELEV, -1000, -1);
    }

    public void moveBot(int distance, double speed) {
        if (Math.signum(distance) != Math.signum(speed))
            speed *= -1;
        move(UP, distance, speed);
        move(LEFT, distance, speed);
        move(DOWN, -distance, -speed);
        move(RIGHT, -distance, -speed);
    }

    public void rotateBot(int distance, double speed) {
        if (Math.signum(distance) != Math.signum(speed))
            speed *= -1;
        move(UP, distance, speed);
        move(LEFT, distance, speed);
        move(DOWN, distance, speed);
        move(RIGHT, distance, speed);
    }

    public void moveBotDiUD(int distance) {
        double speed = DRIVE_SPEED;
        if (Math.signum(distance) != Math.signum(speed))
            speed *= -1;
        move(UP, distance, speed);
        move(DOWN, -distance, -speed);
    }

    public void moveBotDiLR(int distance) {
        double speed = DRIVE_SPEED;
        if (Math.signum(distance) != Math.signum(speed))
            speed *= -1;
        move(LEFT, distance, speed);
        move(RIGHT, -distance, -speed);
    }

    public void rotateBot(int distance) {
        rotateBot(distance, ROTATE_SPEED);
    }
    public void moveBot(int distance) {
        moveBot(distance, DRIVE_SPEED);
    }

    public void lowerJArm() {
        move(JEWEL, -700, 0.2);
    }
    public void raiseJArm() {
        move(JEWEL, 700, -0.2);
    }

    public int getPos(int motor) {
        return motors[motor].getCurrentPosition();
    }

    public void look() throws InterruptedException {
        v.look();
        int i;
        if (v.pictograph == null) {
            move(UP, 50, 0.1);
            move(LEFT, -50, -0.1);
            move(DOWN, 50, 0.1);
            move(RIGHT, -50, -0.1);
            while(motors[UP].isBusy()) {
                v.look();
                if (v.pictograph != null) {
                    break;
                }
            }
            move(UP, 0, 0);
            move(LEFT, 0, 0);
            move(DOWN, 0, 0);
            move(RIGHT, 0, 0);
            if (v.pictograph == null) {
                move(UP, -50, -0.1);
                move(LEFT, 50, 0.1);
                move(DOWN, -50, -0.1);
                move(RIGHT, 50, 0.1);
                while(motors[UP].isBusy()) {
                    v.look();
                    if (v.pictograph != null) {
                        break;
                    }
                }
            }
            move(UP, 0, 0);
            move(LEFT, 0, 0);
            move(DOWN, 0, 0);
            move(RIGHT, 0, 0);
        }
        if (v.pictograph == null) {
            v.blindLook();
            v.pictograph = RelicRecoveryVuMark.CENTER;
        }
    }
    public void reset() {}
}
