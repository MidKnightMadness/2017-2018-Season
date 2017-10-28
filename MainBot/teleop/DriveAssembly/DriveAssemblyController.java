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

            double translateScale = Math.min(Math.max(Math.abs((gamepad1.left_stick_x + gamepad1.left_stick_y)), -0.7), 0.7);
            double targetDirection = aTan(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double targetRotationSpeed = gamepad1.right_stick_x * (1 - Math.abs(translateScale));

            motorUp.setPower(translateScale * Math.cos((theta + targetDirection) * (Math.PI / 180d)) + (targetRotationSpeed));
            motorDown.setPower(-translateScale * Math.cos((theta + targetDirection) * (Math.PI / 180d)) + (targetRotationSpeed));
            motorLeft.setPower(translateScale * Math.sin((theta + targetDirection) * (Math.PI / 180d)) + (targetRotationSpeed));
            motorRight.setPower(-translateScale * Math.sin((theta + targetDirection) * (Math.PI / 180d)) + (targetRotationSpeed));


            telemetry.addData("Theta", theta);
            telemetry.addData("IMU Rotation", getIMURotation());
            telemetry.addData("Start Position", startPos);
            telemetry.addData("Target Direction", targetDirection);
            telemetry.addData("Target Rotation Speed", targetRotationSpeed);
            telemetry.addData("Translate Scale", translateScale);
            telemetry.addData("Cosine", Math.cos((theta + targetDirection) * (Math.PI / 180d)));
            telemetry.addData("Sine", Math.sin((theta + targetDirection) * (Math.PI / 180d)));

            telemetry.update();
        } else {
            motorUp.setPower(gamepad1.left_stick_y);
            motorDown.setPower(-gamepad1.right_stick_y);
            motorLeft.setPower(gamepad1.left_stick_y);
            motorRight.setPower(-gamepad1.right_stick_y);
        }


    }

    public void stop() {}
}
