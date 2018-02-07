package org.firstinspires.ftc.teamcode.MainBot.autonomous.Tests.OlderTests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

//import static org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator.State.homeward;


public class DriveAssemblyController {

    private Telemetry telemetry;

    private BNO055IMU imu;

    private DcMotor motorUp;
    private DcMotor motorDown;
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private Servo vsd;

    private static int BASE_ROTATION_ANGLE = -135;
    private int target = 0;

    private double startPos = 0;
    private double theta = 0;
    private boolean bPressed = false;
    private boolean backPressed = false;
    private boolean yPressed = false;
    private boolean tankMode = false;
    private boolean slow = false;
    private int turnDir = 0;
    private double tempMotors[] = new double[4];
    private double targets[] = new double[3];
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
        motorUp = hardwareMap.dcMotor.get(CrossCommunicator.Drive.FRONT_LEFT);
        motorDown = hardwareMap.dcMotor.get(CrossCommunicator.Drive.BACK_RIGHT);
        motorLeft = hardwareMap.dcMotor.get(CrossCommunicator.Drive.BACK_LEFT);
        motorRight = hardwareMap.dcMotor.get(CrossCommunicator.Drive.FRONT_RIGHT);

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

    private void resetHeading() {
        startPos = getIMURotation() - BASE_ROTATION_ANGLE;
    }

    private void targPow(DcMotor motor, int id, double speed) {
        motor.setPower(speed);
    }

    public void loop(double x, double y, double r) {
        theta = getIMURotation() - startPos;

            //if (Math.abs((theta + 3780)%360 - (target + 3780)%360) > 3) {
            //    adjustedR = Math.min(Math.max(((theta + 3780)%360 - (target + 3780)%360)/10, -0.6), 0.6);
            //} else {
                adjustedR = 0;
            //}

            adjustedX = Math.min(Math.max(x/2, -0.6), 0.6);
            adjustedY = Math.min(Math.max(y/2, -0.6), 0.6);



            double translateScale = Math.pow(Math.hypot(adjustedX, adjustedY), 5) * (1 - Math.min(Math.pow(Math.abs(adjustedR), 2), 0.6)) * (slow ? 0.5 : 1);
            double targetDirection = aTan(x, y);
            double rotateScale = Math.pow(Math.abs(adjustedR), 5) * Math.signum(-adjustedR) * (1 - Math.abs(translateScale)) * (slow ? 0.5 : 1);

            telemetry.addData("AdjustedR", adjustedR);
            telemetry.addData("AdjustedX", adjustedX);
            telemetry.addData("AdjustedY", adjustedY);

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
    }

    public void stop() {}
}
