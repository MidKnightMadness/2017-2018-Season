package org.firstinspires.ftc.teamcode.MainBot.teleop.DriveAssembly;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;


import java.io.FileInputStream;

import static org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator.State.*;


public class DriveAssemblyController {

    private Telemetry telemetry;

    private BNO055IMU imu;

    private DcMotor motorUp;
    private DcMotor motorDown;
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private Servo vsd;

    private static int BASE_ROTATION_ANGLE = -135;
    private int corner = 0;
    private int target = 0;
    private double timeToHomeward = 0;
    private boolean timeForHomeward = false;
    private static int[][] targets = new int[][]{
            //When Left (0) or When Right (1)?
            //RedRecovery
            {60, 30},
            //RedNonRecovery
            {-45, -45},
            //BlueRecovery
            {60, 30},
            //BlueNonRecovery
            {-45, -45}
    };

    private double startPos = 0;
    private double theta = 0;
    private boolean bPressed = false;
    private boolean backPressed = false;
    private boolean yPressed = false;
    private boolean tankMode = false;
    private boolean slow = false;
    private int turnDir = 0;
    private double tempMotors[] = new double[4];
    private double adjustedX = 0;
    private double adjustedY = 0;
    private double adjustedR = 0;

    //Gets angle of vector <x,y>
    private double aTan(double x, double y) {
        double a = 0;
        if (y == 0) {
            if (x < 0) {
                a = -90;
            } else if (x > 0) {
                a = 90;
            } else if (x == 0) {
                a = 0;
            }
        } else if (x == 0) {
            a = 0;
        } else {
            a = (Math.atan(x/y)*(180/Math.PI));
        }

        if (y < 0) {
            if (x < 0) {
                a -= 180;
            } else {
                a += 180;
            }
        }
        return a;
    }


    private double getIMURotation() {
        return (AngleUnit.normalizeDegrees(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle));
    }

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;

        //Init IMU with parameters and start integration
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 50);

        //Init motors
        motorUp = hardwareMap.dcMotor.get(CrossCommunicator.Drive.UP);
        motorDown = hardwareMap.dcMotor.get(CrossCommunicator.Drive.DOWN);
        motorLeft = hardwareMap.dcMotor.get(CrossCommunicator.Drive.LEFT);
        motorRight = hardwareMap.dcMotor.get(CrossCommunicator.Drive.RIGHT);

        motorUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetHeading();

        vsd = hardwareMap.servo.get("vsd");
        readTeamColor();
    }

    public void start() {vsd.setPosition(1);}

    private void resetHeading() {
        startPos = getIMURotation() - BASE_ROTATION_ANGLE;
    }

    private void toggleTankMode() {
        tankMode = !tankMode;
    }

    private void targPow(DcMotor motor, int id, double speed) {
        motor.setPower(speed);
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        target = targets[corner][turnDir];

        if ((gamepad1.a || gamepad2.a)) {
            resetHeading();
        }

        if ((gamepad1.b || gamepad2.b )&& !bPressed) {
            toggleTankMode();
            bPressed = true;
        } else if (!(gamepad1.b|| gamepad2.b)){
            bPressed = false;
        }

        if ((gamepad1.back || gamepad2.back )&& !backPressed) {
            slow = !slow;
            backPressed = true;
        } else if (!(gamepad1.back|| gamepad2.back)){
            backPressed = false;
        }

        if ((gamepad1.y || gamepad2.y )&& !yPressed) {
            homeward = !homeward;
            if (homeward) {
                timeForHomeward = false;
                timeToHomeward = time.seconds() + 1d;
            }
            justChanged = true;
            yPressed = true;
        } else if (!(gamepad1.y|| gamepad2.y)){
            yPressed = false;
        }

        if ((gamepad1.dpad_left|| gamepad2.dpad_left)) {
            turnDir = 0;
        } else if ((gamepad1.dpad_right|| gamepad2.dpad_right)) {
            turnDir = 1;
        }

        if (time.seconds() > timeToHomeward) {
            timeForHomeward = true;
        }

        if (!tankMode) {
            theta = getIMURotation() - startPos;
            adjustedX = gamepad1.left_stick_x;
            adjustedY = gamepad1.left_stick_y;
            adjustedR = gamepad1.right_stick_x;

            if (Math.abs(adjustedR) > 0.1 && homeward && timeForHomeward) {
                timeToHomeward = Integer.MAX_VALUE;
                timeForHomeward = false;
            }

            if (homeward && timeForHomeward && Math.abs((theta - target + 3780)%360 - 180) > 10) {
                adjustedR = Math.min(Math.max(((theta - target + 3780)%360 - 180)/30, -1), 1);
            } else if (homeward && timeForHomeward) {
                adjustedR = 0;
            }

            double translateScale = Math.pow(Math.hypot(adjustedX, adjustedY), 5) * (1 - Math.min(Math.pow(Math.abs(adjustedR), 2), 0.6)) * (slow ? 0.5 : 1);
            double targetDirection = aTan(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double rotateScale = Math.pow(Math.abs(adjustedR), 5) * Math.signum(-adjustedR) * (1 - Math.abs(translateScale)) * (slow ? 0.5 : 1);

            telemetry.addData("Offset", (theta - target + 3780)%360 - 180);
            telemetry.addData("Needs To Move", Math.abs((theta - target + 3780)%360 - 180) > 10);
            telemetry.addData("Theta", theta);
            telemetry.addData("Target", target);
            telemetry.addData("Homeward", homeward);

            telemetry.addData("Rotate Scale", rotateScale);

            tempMotors[0] = Math.cos((theta + targetDirection) * (Math.PI / 180d));
            tempMotors[1] = -Math.cos((theta + targetDirection) * (Math.PI / 180d));
            tempMotors[2] = Math.sin((theta + targetDirection) * (Math.PI / 180d));
            tempMotors[3] = -Math.sin((theta + targetDirection) * (Math.PI / 180d));

            double scale = Math.max(Math.max(Math.abs(tempMotors[0]), Math.abs(tempMotors[1])), Math.max(Math.abs(tempMotors[2]), Math.abs(tempMotors[3])));
            if (scale == 0) {
                scale = 0;
            } else {
                scale = translateScale/(scale);
            }

            tempMotors[0] *= scale;
            tempMotors[1] *= scale;
            tempMotors[2] *= scale;
            tempMotors[3] *= scale;

            tempMotors[0] += rotateScale;
            tempMotors[1] += rotateScale;
            tempMotors[2] += rotateScale;
            tempMotors[3] += rotateScale;

            targPow(motorUp, 0, tempMotors[0]);
            targPow(motorDown, 1, tempMotors[1]);
            targPow(motorLeft, 2, tempMotors[2]);
            targPow(motorRight, 3, tempMotors[3]);

            telemetry.update();
            vsd.setPosition(1);
        } else {
            if (Math.abs(gamepad1.left_stick_x + gamepad1.right_stick_x) > 1.5) {
                targPow(motorUp, 0, -gamepad1.left_stick_x);
                targPow(motorDown, 1, gamepad1.left_stick_x);
                targPow(motorLeft, 2, gamepad1.left_stick_x);
                targPow(motorRight, 3, -gamepad1.left_stick_x);
            } else {
                targPow(motorUp, 0, Math.pow(gamepad1.left_stick_y, 3));
                targPow(motorDown, 1, -Math.pow(gamepad1.right_stick_y, 3));
                targPow(motorLeft, 2, Math.pow(gamepad1.left_stick_y, 3));
                targPow(motorRight, 3, -Math.pow(gamepad1.right_stick_y, 3));
            }
            vsd.setPosition(0.5);
        }
    }

    public void stop() {}

    private void readTeamColor() {
        try {
            FileInputStream f = new FileInputStream("/storage/self/primary/LastTeamColor.txt");
            if (f.available() >= 1) {
                corner = f.read();
            }
        } catch (Exception e) {
            telemetry.addData("Unable to read config. Error: ", e);
            telemetry.update();
        }
    }
}
