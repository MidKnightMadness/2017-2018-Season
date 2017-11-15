package org.firstinspires.ftc.teamcode.MainBot.teleop.DriveAssembly;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;
//AAUNIOEFIPSJNUPIUAENHPDfbj9w9ER -Your favorite annoyance
public class DriveAssemblyController {

    private Telemetry telemetry;

    private BNO055IMU imu;

    private DcMotor motorUp;
    private DcMotor motorDown;
    private DcMotor motorLeft;
    private DcMotor motorRight;

    private static int BASE_ROTATION_ANGLE = 135;

    private double startPos = 0;
    private double theta = 0;
    private boolean bPressed = false;
    private boolean tankMode = false;
    private double motors[] = new double[4];
    private double mainTranslateScale = 0;
    private double mainRotateScale = 0;

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
    }

    public void start() {}

    private void resetHeading() {
        startPos = getIMURotation() - BASE_ROTATION_ANGLE;
    }

    private void toggleTankMode() {
        tankMode = !tankMode;
    }

    private void targPow(DcMotor motor, int id, double speed) {
        if ((speed - motors[id]) < 0.05)
            motors[id] = speed;
        else
            motors[id] += Math.signum(speed - motors[id])*0.1;
        motor.setPower(speed);
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad1.a) {
            resetHeading();
        }

        if (gamepad1.b && !bPressed) {
            toggleTankMode();
            bPressed = true;
        } else if (!gamepad1.b){
            bPressed = false;
        }

        if (!tankMode) {
            theta = getIMURotation() - startPos;

            double translateScale = Math.pow(Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y), 2) * (1 - Math.min(Math.pow(Math.abs(gamepad1.right_stick_x), 2), 0.5));
            if ((translateScale - mainTranslateScale) < 0.1)
                mainTranslateScale = translateScale;
            else
                mainTranslateScale += Math.signum(translateScale - mainTranslateScale)*0.1;


            double targetDirection = aTan(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double rotateScale = Math.pow(Math.abs(gamepad1.right_stick_x), 2) * Math.signum(-gamepad1.right_stick_x) * (1 - Math.abs(translateScale));
            if ((rotateScale - mainRotateScale) < 0.1)
                mainRotateScale = rotateScale;
            else
                mainRotateScale += Math.signum(rotateScale - mainRotateScale)*0.1;

            targPow(motorUp, 0, mainTranslateScale * Math.cos((theta + targetDirection) * (Math.PI / 180d)) + (rotateScale));
            targPow(motorDown, 1, -mainTranslateScale * Math.cos((theta + targetDirection) * (Math.PI / 180d)) + (rotateScale));
            targPow(motorLeft, 2, mainTranslateScale * Math.sin((theta + targetDirection) * (Math.PI / 180d)) + (rotateScale));
            targPow(motorRight, 3, -mainTranslateScale * Math.sin((theta + targetDirection) * (Math.PI / 180d)) + (rotateScale));


            telemetry.addData("Theta", theta);
            telemetry.addData("IMU Rotation", getIMURotation());
            telemetry.addData("Start Position", startPos);
            telemetry.addData("Target Direction", targetDirection);
            telemetry.addData("Target Rotation Speed", rotateScale);
            telemetry.addData("Translate Scale", translateScale);
            telemetry.addData("Cosine", Math.cos((theta + targetDirection) * (Math.PI / 180d)));
            telemetry.addData("Sine", Math.sin((theta + targetDirection) * (Math.PI / 180d)));

            telemetry.update();
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
        }


    }

    public void stop() {}
}
