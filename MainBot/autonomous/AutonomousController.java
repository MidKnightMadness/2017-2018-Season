package org.firstinspires.ftc.teamcode.MainBot.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;


public class AutonomousController {

    //TODO: CHECK WIRING ON JEWEL ARM: REVERSED? WHY REVERSED SPEED AND ENCODERS??

    public static int GLYPH = 0;
    public static int JEWEL = 1;
    public static int UP = 2;
    public static int DOWN = 3;
    public static int LEFT = 4;
    public static int RIGHT = 5;

    private static double DRIVE_SPEED = 0.4;
    private static double ROTATE_SPEED = 0.3;



    public Servo glyphServo;
    public int[] pos = new int[6];
    public DcMotor[] motors = new DcMotor[6];

    private static VisualController.JewelColor TEAM_COLOR = VisualController.JewelColor.RED;

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        motors[GLYPH] = hardwareMap.dcMotor.get(CrossCommunicator.Glyph.MOTOR);
        motors[GLYPH].resetDeviceConfigurationForOpMode();
        motors[GLYPH].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors[GLYPH].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motors[JEWEL] = hardwareMap.dcMotor.get(CrossCommunicator.Jewel.MOTOR);
        motors[JEWEL].resetDeviceConfigurationForOpMode();
        motors[JEWEL].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors[JEWEL].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motors[UP] = hardwareMap.dcMotor.get(CrossCommunicator.Drive.UP);
        motors[UP].resetDeviceConfigurationForOpMode();
        motors[UP].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors[UP].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motors[DOWN] = hardwareMap.dcMotor.get(CrossCommunicator.Drive.DOWN);
        motors[DOWN].resetDeviceConfigurationForOpMode();
        motors[DOWN].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors[DOWN].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motors[LEFT] = hardwareMap.dcMotor.get(CrossCommunicator.Drive.LEFT);
        motors[LEFT].resetDeviceConfigurationForOpMode();
        motors[LEFT].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors[LEFT].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motors[RIGHT] = hardwareMap.dcMotor.get(CrossCommunicator.Drive.RIGHT);
        motors[RIGHT].resetDeviceConfigurationForOpMode();
        motors[RIGHT].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors[RIGHT].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        glyphServo = hardwareMap.servo.get(CrossCommunicator.Glyph.SERVO);
        glyphServo.setPosition(1);

        reset();
    }

    private void move(int motor, int relTarget, double speed) {
        motors[motor].setTargetPosition(motors[motor].getCurrentPosition() + relTarget);
        motors[motor].setPower(-speed);
    }

    public void close() {
        glyphServo.setPosition(0);
    }
    public void open() {
        glyphServo.setPosition(1);
    }
    public void lift() {
        move(GLYPH, 1000, 1);
    }
    public void lower() {
        move(GLYPH, -1000, -1);
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
        move(JEWEL, -600, 0.3);
    }

    public void raiseJArm() {
        move(JEWEL, 600, -0.3);
    }

    public int getPos(int motor) {
        return motors[motor].getCurrentPosition();
    }

    public void reset() {
        //pos[GLYPH] = motors[GLYPH].getCurrentPosition();
        //pos[JEWEL] = motors[JEWEL].getCurrentPosition();
        //pos[UP] = motors[UP].getCurrentPosition();
        //pos[DOWN] = motors[DOWN].getCurrentPosition();
        //pos[LEFT] = motors[LEFT].getCurrentPosition();
        //pos[RIGHT] = motors[RIGHT].getCurrentPosition();
    }
}
