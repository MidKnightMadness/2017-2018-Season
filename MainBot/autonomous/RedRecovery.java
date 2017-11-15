package org.firstinspires.ftc.teamcode.MainBot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

@Autonomous(name = "RedRecovery", group = "Main Bot")
public class RedRecovery extends LinearOpMode {
    private static VisualController.JewelColor TEAM_COLOR = VisualController.JewelColor.RED;
    private static double JEWEL_ARM_POWER = 0.3;
    private static int JEWEL_ARM_DISTANCE = 600;
    private static double DRIVE_ROTATE_POWER = -0.3;
    private static int DRIVE_ROTATE_DISTANCE = 200;
    private static double DRIVE_MOVE_POWER = 0.4;
    private static int DRIVE_MOVE_DISTANCE = 2500;
    private static int DRIVE_ROTATE90_DISTANCE = 1523;
    private DcMotor jewelMotor;
    private DcMotor driveUpMotor;
    private DcMotor driveDownMotor;
    private DcMotor driveLeftMotor;
    private DcMotor driveRightMotor;
    private VisualController visualC = new VisualController();
    private GlyphController glyphC = new GlyphController();


    @Override
    public void runOpMode() throws InterruptedException {
        initialize(hardwareMap);

        visualC.init(telemetry, hardwareMap);
        glyphC.init(telemetry, hardwareMap);

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();

        glyphC.grab();
        glyphC.lift();
        visualC.look();
        lowerArm();
        rotateBot(true);
        raiseArm();
        rotateBot(false);
        if (visualC.pictograph == RelicRecoveryVuMark.RIGHT) {
            DRIVE_MOVE_DISTANCE = 1850;
        }
        else if (visualC.pictograph == RelicRecoveryVuMark.LEFT){
            DRIVE_MOVE_DISTANCE = 3150;
        }
        else {
            DRIVE_MOVE_DISTANCE = 2500;
        }
        telemetry.addLine("Distance: " + DRIVE_MOVE_DISTANCE);
        telemetry.update();
        /*double delay = time + 1;
        while (time < delay) {
            idle();
        }*/
        moveBot();
        speedRotateBot(-0.3, DRIVE_ROTATE90_DISTANCE);
        DRIVE_ROTATE90_DISTANCE = 1523;

        glyphC.drop();


    }


    void initialize(HardwareMap hardwareMap) {
        jewelMotor = hardwareMap.dcMotor.get(CrossCommunicator.Jewel.MOTOR);
        jewelMotor.resetDeviceConfigurationForOpMode();
        jewelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveUpMotor = hardwareMap.dcMotor.get(CrossCommunicator.Drive.UP);
        driveUpMotor.resetDeviceConfigurationForOpMode();
        driveUpMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveUpMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveDownMotor = hardwareMap.dcMotor.get(CrossCommunicator.Drive.DOWN);
        driveDownMotor.resetDeviceConfigurationForOpMode();
        driveDownMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveDownMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveLeftMotor = hardwareMap.dcMotor.get(CrossCommunicator.Drive.LEFT);
        driveLeftMotor.resetDeviceConfigurationForOpMode();
        driveLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveRightMotor = hardwareMap.dcMotor.get(CrossCommunicator.Drive.RIGHT);
        driveRightMotor.resetDeviceConfigurationForOpMode();
        driveRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    void lowerArm() {
        telemetry.addLine("Lowering Arm...");
        telemetry.update();

        jewelMotor.setTargetPosition(jewelMotor.getCurrentPosition() - JEWEL_ARM_DISTANCE);
        jewelMotor.setPower(JEWEL_ARM_POWER);
        while (jewelMotor.isBusy()) {
            idle();
        }
    }

    void raiseArm() {
        telemetry.addLine("Raise Arm...");
        telemetry.update();

        jewelMotor.setTargetPosition(jewelMotor.getCurrentPosition() + JEWEL_ARM_DISTANCE);
        jewelMotor.setPower(-JEWEL_ARM_POWER); // TODO: why negative if distance positive?
        while (jewelMotor.isBusy()) {
            idle();
        }
    }

    void speedRotateBot(double speed, int driveRotateDistance) {
        telemetry.addLine("Rotate Bot " + speed + " " + driveRotateDistance);
        telemetry.update();

        driveUpMotor.setTargetPosition(driveUpMotor.getCurrentPosition() + (driveRotateDistance));
        driveUpMotor.setPower(speed);

        driveDownMotor.setTargetPosition(driveDownMotor.getCurrentPosition() + (driveRotateDistance));
        driveDownMotor.setPower(speed);

        driveLeftMotor.setTargetPosition(driveLeftMotor.getCurrentPosition() + (driveRotateDistance));
        driveLeftMotor.setPower(speed);

        driveRightMotor.setTargetPosition(driveRightMotor.getCurrentPosition() + (driveRotateDistance));
        driveRightMotor.setPower(speed);

        while (driveUpMotor.isBusy()) {
            idle();
        }

    }

    void rotateBot(Boolean reset) {
        telemetry.addLine("Rotate Bot " + (reset ? "First..." : "Second..."));
        telemetry.update();

        int neg = (visualC.leftJewel == TEAM_COLOR) ? 1 : -1;
        neg = (reset) ? neg : -neg;

        driveUpMotor.setTargetPosition(driveUpMotor.getCurrentPosition() + (neg * DRIVE_ROTATE_DISTANCE));
        driveUpMotor.setPower(neg * DRIVE_ROTATE_POWER);

        driveDownMotor.setTargetPosition(driveDownMotor.getCurrentPosition() + (neg * DRIVE_ROTATE_DISTANCE));
        driveDownMotor.setPower(neg * DRIVE_ROTATE_POWER);

        driveLeftMotor.setTargetPosition(driveLeftMotor.getCurrentPosition() + (neg * DRIVE_ROTATE_DISTANCE));
        driveLeftMotor.setPower(neg * DRIVE_ROTATE_POWER);

        driveRightMotor.setTargetPosition(driveRightMotor.getCurrentPosition() + (neg * DRIVE_ROTATE_DISTANCE));
        driveRightMotor.setPower(neg * DRIVE_ROTATE_POWER);

        while (driveUpMotor.isBusy()) {
            idle();
        }

    }

    void moveBot() {
        telemetry.addLine("Move Bot...");
        telemetry.update();

        driveUpMotor.setTargetPosition(driveUpMotor.getCurrentPosition() + DRIVE_MOVE_DISTANCE);
        driveUpMotor.setPower(DRIVE_MOVE_POWER);

        driveDownMotor.setTargetPosition(driveDownMotor.getCurrentPosition() - DRIVE_MOVE_DISTANCE);
        driveDownMotor.setPower(-DRIVE_MOVE_POWER);

        driveLeftMotor.setTargetPosition(driveLeftMotor.getCurrentPosition() + DRIVE_MOVE_DISTANCE);
        driveLeftMotor.setPower(DRIVE_MOVE_POWER);

        driveRightMotor.setTargetPosition(driveRightMotor.getCurrentPosition() - DRIVE_MOVE_DISTANCE);
        driveRightMotor.setPower(-DRIVE_MOVE_POWER);

        while (driveUpMotor.isBusy()) {
            idle();
        }
    }
}