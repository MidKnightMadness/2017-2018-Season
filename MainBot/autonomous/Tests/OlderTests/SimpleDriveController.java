package org.firstinspires.ftc.teamcode.MainBot.autonomous.Tests.OlderTests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

/**
 * Created by gregory.ling on 10/24/17.
 */

public class SimpleDriveController {
    private DcMotor motorUp;
    private DcMotor motorDown;
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private double x;
    private double y;
    private double r;

    private BNO055IMU imu;

    public void init(HardwareMap hardwareMap) {
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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 50);
    }

    private double getIMURotation() {
        return (AngleUnit.normalizeDegrees(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle) + 3600)%360;
    }

    public void rotate(double r) {
        this.r = r;
        motorUp.setPower(r);
        motorDown.setPower(r);
        motorLeft.setPower(r);
        motorRight.setPower(r);
    }

    public void translate(double x, double y) {
        this.x = x;
        this.y = y;
        motorUp.setPower(x);
        motorDown.setPower(-x);
        motorLeft.setPower(y);
        motorRight.setPower(-y);
    }

    public boolean isBusyR(int rT) {
        return !(Math.abs(getIMURotation() - rT) < 10);
    }

    public boolean isBusyT(int xT, int yT) {
        if (Math.signum(motorUp.getCurrentPosition() - xT) != Math.signum(x)) {
            translate(-0.5 * x, y);
        } else if (Math.signum(motorLeft.getCurrentPosition() - yT) != Math.signum(y)) {
            translate(x, -0.5 * y);
        }

        return !(Math.abs(motorUp.getCurrentPosition() - xT) + Math.abs(motorLeft.getCurrentPosition() - yT) < 10);
    }
}
