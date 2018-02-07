package org.firstinspires.ftc.teamcode.MainBot.teleop.DriveAssembly.Tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

/**
 * Created by gregory.ling on 10/17/17.
 */

public class DriveAssemblyNotSoOldController {
    private static boolean MOTORS_ENABLED = true;
    private static double TURN_SPEED_RATIO = 2;
    private double motorDriverSpeed = 0.8;
    private boolean isUsingTheta = true;
    private boolean bPressed = false;
    private boolean yPressed = false;

    private Telemetry telemetry;
    private DcMotor motorUp;
    private DcMotor motorDown;
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private double motorSpeeds[] = new double[4];
    private double theta = 135;
    private BNO055IMU imu;
    private double startPos = 0;
    private int[] oldMotorPos = new int[4];

    private double targetSpeed = 0;
    private double targetDistance = 0;
    private double targetDirection = 0;
    private double targetRotation = 0;
    private double targetRotationSpeed = 0;
    public boolean reachedTargetRotation = true;
    public boolean reachedTargetTranslation = true;
    private boolean isDriverControlled = false;

    private VectorF netTranslationVector = new VectorF(0, 0);
    private double netRotOffset = 0;

    private double ly = 0;
    private double ry = 0;


    /**
     * The init phase. Run before using any other functions in this class.
     * @param telemetry    the telemetry instance
     * @param hardwareMap  the hardwareMap instance
     */
    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;

        // INIT IMU: angle -> Degrees  and initialize     (No calibration necessary)
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Init motors if enabled (get motor from hardware map; set mode to using encoder; set to brake when stopped)
        if (MOTORS_ENABLED) {
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
        }
    }

    private double getIMURotation() {
        return AngleUnit.normalizeDegrees(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
    }

    /**
     * An internal function to get the angle of the joystick; an expanded arctan
     * @param x  the joystick x
     * @param y  the joystick y
     */
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

    //on start, set startPos to theta
    public void start() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 50);
        resetTheta(135);
        oldMotorPos = new int[]{motorUp.getCurrentPosition(), motorDown.getCurrentPosition(), motorLeft.getCurrentPosition(), motorRight.getCurrentPosition()};
    }

    /**
     * Reset theta (the robot's current rotation) to the angle provided.
     * @param reset  the angle to reset to
     */
    public void resetTheta(double reset) {
        startPos = getIMURotation() + reset;
        theta = reset;
    }

    /**
     * Reset the totalTranslationVector to 0,0
     */
    public void resetNetTransVect() {
        netTranslationVector = new VectorF(0, 0);
    }

    /**
     * Reset the totalTranslationVector to 0,0
     */
    public VectorF getNetTransVect() {
        return netTranslationVector;
    }

    /**
     * Reset the net rotation. Does not reset startPos.
     */
    public void resetNetRotation() {
        netRotOffset = theta - startPos;
    }

    /**
     * Get the net Rotation since reset
     */
    public double getNetRotation() {
        return (theta - startPos) - netRotOffset;
    }

    /**
     * Set the rotation mode (Intrinsic (vehicle coordinates) or Extrinsic (world coordinates))
     * @param isNowUsingTheta  whether or not we are now using theta (true is extrinsic)
     */
    public void setRotationMode(boolean isNowUsingTheta) {
        isUsingTheta = isNowUsingTheta;
        if (isUsingTheta) {
            resetTheta(135);
        }
    }

    public boolean getRotationMode() {
        return isUsingTheta;
    }

    /**
     * Set the target movement of the robot
     * @param speed             the speed from -1 to 1 to which the motor values are scaled
     * @param distance          the distance the robot should move (encoder-counts). Set to zero if not translating.
     * @param direction         the direction (degrees) the robot should move at
     * @param rotation          the target degrees to which the robot is moving; ignored if driverControlled is true
     * @param rotationSpeed     the speed at which to rotate the robot (scaled -1 to 1)
     * @param driverControlled  whether or not the robot is currently driver-controlled
     */
    public void setTarget(double speed, double distance, double direction, double rotation, double rotationSpeed, boolean driverControlled) {
        reachedTargetRotation = false;
        reachedTargetTranslation = false;

        targetSpeed = speed;
        targetDistance = distance;
        targetDirection = direction;
        targetRotation = theta + rotation;
        targetRotationSpeed = rotationSpeed * (theta - targetRotation > 0 ? 1 : -1);
        isDriverControlled = driverControlled;
    }

    /**
     * Updates the motor speeds based on the current targets
     */
    public void update() {
        if (!isDriverControlled) {
            telemetry.addData("Diff: ", (theta - targetRotation));
            if (Math.abs((theta - targetRotation + 3600)%360) < 3) {
                reachedTargetRotation = true;
                targetRotationSpeed = 0;
            }
            if (Math.signum(theta - targetRotation) == Math.signum(targetRotationSpeed)) {
                targetRotationSpeed = -0.5*targetRotationSpeed;
            }
            targetDistance -= Math.hypot(((motorLeft.getCurrentPosition() - oldMotorPos[2]) - (motorRight.getCurrentPosition() - oldMotorPos[3]) / 2), ((motorUp.getCurrentPosition() - oldMotorPos[0]) - (motorDown.getCurrentPosition() - oldMotorPos[1]) / 2));
            if (targetDistance <= 0) {
                targetDistance = 0;
                reachedTargetTranslation = true;
            }
        }

        netTranslationVector.add(new VectorF(((motorLeft.getCurrentPosition() - oldMotorPos[2]) - (motorRight.getCurrentPosition() - oldMotorPos[3]) / 2), ((motorUp.getCurrentPosition() - oldMotorPos[0]) - (motorDown.getCurrentPosition() - oldMotorPos[1]) / 2)));
        oldMotorPos = new int[]{motorUp.getCurrentPosition(), motorDown.getCurrentPosition(), motorLeft.getCurrentPosition(), motorRight.getCurrentPosition()};

        if (isUsingTheta) {
            theta = (getIMURotation() - startPos + 360000) % 360;


            int isTranslating = (targetDistance == 0 ? 0 : 1);
            motorSpeeds[0] = isTranslating * Math.cos((theta + targetDirection) * (Math.PI / 180d)) + (targetRotationSpeed * TURN_SPEED_RATIO);
            motorSpeeds[1] = -isTranslating * Math.cos((theta + targetDirection) * (Math.PI / 180d)) + (targetRotationSpeed * TURN_SPEED_RATIO);
            motorSpeeds[2] = isTranslating * Math.sin((theta + targetDirection) * (Math.PI / 180d)) + (targetRotationSpeed * TURN_SPEED_RATIO);
            motorSpeeds[3] = -isTranslating * Math.sin((theta + targetDirection) * (Math.PI / 180d)) + (targetRotationSpeed * TURN_SPEED_RATIO);


            double scale = Math.max(Math.max(Math.abs(motorSpeeds[0]), Math.abs(motorSpeeds[1])), Math.max(Math.abs(motorSpeeds[2]), Math.abs(motorSpeeds[3])));
            if (scale == 0) {
                scale = 0;
            } else {
                scale = targetSpeed / scale;
            }

            motorSpeeds[0] *= scale;
            motorSpeeds[1] *= scale;
            motorSpeeds[2] *= scale;
            motorSpeeds[3] *= scale;
        } else {
            motorSpeeds[0] = ry;
            motorSpeeds[1] = -ly;
            motorSpeeds[2] = ry;
            motorSpeeds[3] = -ly;
        }

        telemetry.addLine("Theta: " + (theta - startPos));
        telemetry.addData("TargetTheta: ", Math.abs((theta - targetRotation + 36000)%360));
        telemetry.addLine("Up Motor Speed: " + motorSpeeds[0]);
        telemetry.addLine("Down Motor Speed: " + motorSpeeds[1]);
        telemetry.addLine("Left Motor Speed: " + motorSpeeds[2]);
        telemetry.addLine("Right Motor Speed: " + motorSpeeds[3]);

        if (MOTORS_ENABLED) {
            motorUp.setPower(motorSpeeds[0]);
            motorDown.setPower(motorSpeeds[1]);
            motorLeft.setPower(motorSpeeds[2]);
            motorRight.setPower(motorSpeeds[3]);
        }
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad1.a) {
            resetTheta(135);
        }
        if (gamepad1.b && !bPressed) {
            setRotationMode(isUsingTheta);
            bPressed = true;
        } else if (!gamepad1.b){
            bPressed = false;
        }
        if (gamepad1.x) {
            resetNetTransVect();
            resetNetRotation();
        }
        if (gamepad1.y && !yPressed) {
            motorDriverSpeed = (motorDriverSpeed == 0.8 ? 0.4 : 0.8);
            yPressed = true;
        } else if (!gamepad1.y){
            yPressed = false;
        }

        ly = gamepad1.left_stick_y;
        ry = gamepad1.right_stick_y;
        setTarget(motorDriverSpeed, ((gamepad1.left_stick_x == 0 && -gamepad1.left_stick_y == 0) ? 0 : 1), aTan(gamepad1.left_stick_x, -gamepad1.left_stick_y), 1, gamepad1.right_stick_x, true);
        update();
        telemetry.addData("Current Translation Vector: ", getNetTransVect());
        telemetry.addData("Net Translation Direction: ", aTan(getNetTransVect().get(0), getNetTransVect().get(1)));
        telemetry.addData("Net Distance Travelled: ", getNetTransVect().magnitude());
        telemetry.addData("Net Rotation: ", getNetRotation());
        telemetry.update();
    }

    public void stop() {
    }
}
