package org.firstinspires.ftc.teamcode.MainBot.autonomous.Tests.OlderTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.MainBot.autonomous.VisualController;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

@Disabled
@Autonomous(name = "RedRecoveryAutonomous", group = "Main Bot")
public class RedRecoveryAutonomous extends LinearOpMode {
    private static VisualController.JewelColor TEAM_COLOR = VisualController.JewelColor.RED;
    private int glyphStartPosition;
    private DcMotor jewelMotor;
    private DcMotor glyphMotor;
    private Servo glyphServo;
    private DcMotor driveUpMotor;
    private DcMotor driveDownMotor;
    private DcMotor driveLeftMotor;
    private DcMotor driveRightMotor;
    private VisualController visualC = new VisualController();
    private final int DRIVE_ROTATE90_DISTANCE = 1523;
    private boolean pictographIsUknown = true;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();

        initialize(hardwareMap);
        visualC.init(telemetry, hardwareMap);

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();
        grabGlyph();
        positionBotToViewPictograph();
        knockJewel();
        if (senseRecoveryVuMark() == RelicRecoveryVuMark.RIGHT) {
            placeGlyphInRightColumnAndParkInSafeZone();
        }
        else if (senseRecoveryVuMark() == RelicRecoveryVuMark.LEFT) {
            placeGlyphInLeftColumnAndParkInSafeZone();
        }
        else {
            placeGlyphInCenterColumnAndParkInSafeZone();
        }
    }


    private void initialize(HardwareMap hardwareMap) {
        jewelMotor = hardwareMap.dcMotor.get(CrossCommunicator.Jewel.MOTOR);
        jewelMotor.resetDeviceConfigurationForOpMode();
        jewelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jewelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        glyphMotor = hardwareMap.dcMotor.get(CrossCommunicator.Glyph.ELEV);
        glyphMotor.resetDeviceConfigurationForOpMode();
        glyphMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        glyphMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        glyphMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        glyphStartPosition = glyphMotor.getCurrentPosition();

        glyphServo = hardwareMap.servo.get(CrossCommunicator.Glyph.GRAB_UPPER);

        driveUpMotor = hardwareMap.dcMotor.get(CrossCommunicator.Drive.FRONT_LEFT);
        driveUpMotor.resetDeviceConfigurationForOpMode();
        driveUpMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveUpMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveUpMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveDownMotor = hardwareMap.dcMotor.get(CrossCommunicator.Drive.BACK_RIGHT);
        driveDownMotor.resetDeviceConfigurationForOpMode();
        driveDownMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveDownMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveDownMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveLeftMotor = hardwareMap.dcMotor.get(CrossCommunicator.Drive.BACK_LEFT);
        driveLeftMotor.resetDeviceConfigurationForOpMode();
        driveLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveRightMotor = hardwareMap.dcMotor.get(CrossCommunicator.Drive.FRONT_RIGHT);
        driveRightMotor.resetDeviceConfigurationForOpMode();
        driveRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void grabGlyph() {
        // Close Claw
        telemetry.addLine("Close Claw...");
        telemetry.update();
        glyphServo.setPosition(0);


        wait(0.6);


        // Raise Glyph
        telemetry.addLine("Raise Glyph...");
        telemetry.update();
        glyphMotor.setTargetPosition(glyphStartPosition + 1000);
        glyphMotor.setPower(-1);
    }

    private void placeGlyph() {
        // lower glyph
        telemetry.addLine("Lower Glyph...");
        telemetry.update();
        glyphMotor.setTargetPosition(glyphStartPosition - 1000);
        glyphMotor.setPower(1);

        wait(1.0);

        // release glyph
        telemetry.addLine("Release Glyph...");
        telemetry.update();
        glyphServo.setPosition(0.6);

        wait(0.8);
    }

    private void knockJewel() throws InterruptedException {
        final double DRIVE_ROTATE_POWER = -0.3;
        final int DRIVE_ROTATE_DISTANCE = 250;
        final double JEWEL_ARM_POWER = 0.3;
        final int JEWEL_ARM_DISTANCE = 600;

        int neg = (senseLeftJewelColor() == TEAM_COLOR) ? 1 : -1;

        // lower arm
        telemetry.addLine("Lowering Arm...");
        telemetry.update();
        jewelMotor.setTargetPosition(jewelMotor.getCurrentPosition() - JEWEL_ARM_DISTANCE);
        jewelMotor.setPower(JEWEL_ARM_POWER);
        while (jewelMotor.isBusy()) {
            idle();
        }

        // rotate bot
        rotateBot(neg * DRIVE_ROTATE_POWER,  neg * DRIVE_ROTATE_DISTANCE);

        // raise arm
        telemetry.addLine("Raise Arm...");
        telemetry.update();
        jewelMotor.setTargetPosition(jewelMotor.getCurrentPosition() + JEWEL_ARM_DISTANCE);
        jewelMotor.setPower(-JEWEL_ARM_POWER);
        while (jewelMotor.isBusy()) {
            idle();
        }

        // unrotate bot
        rotateBot(-neg * DRIVE_ROTATE_POWER,  -neg * DRIVE_ROTATE_DISTANCE);
    }

    private void positionBotToViewPictograph() {
        final int DRIVE_MOVE_SIDE_DISTANCE = -75;
        final double DRIVE_MOVE_POWER = 0.4;
        moveBotSide(DRIVE_MOVE_POWER, DRIVE_MOVE_SIDE_DISTANCE);
    }

    // move from balancing stone to and rotate to position the glyph in the right column
    private void placeGlyphInRightColumnAndParkInSafeZone() {
        final double DRIVE_MOVE_POWER = 0.4;
        final int CryptoboxDistance = 1250;
        final int horizontalDistance = 2600;
        final int rotate = DRIVE_ROTATE90_DISTANCE / 2;
        final int direction = -1;

        moveBotSide(DRIVE_MOVE_POWER, horizontalDistance);
        rotateBot(0.3, rotate);
        moveBotDiag(DRIVE_MOVE_POWER, CryptoboxDistance, direction);

        placeGlyph();

        // position bot for teleop
        final int safeZoneDistance = -700;
        moveBotDiag(DRIVE_MOVE_POWER, safeZoneDistance, direction);
    }

    // move from balancing stone to and rotate to position the glyph in the left column
    private void placeGlyphInLeftColumnAndParkInSafeZone() {
        final double DRIVE_MOVE_POWER = 0.4;
        final int CryptoboxDistance = 1250;
        final int horizontalDistance = 2700;
        final int rotate = DRIVE_ROTATE90_DISTANCE * 3 / 2;
        final int direction = 1;

        moveBotSide(DRIVE_MOVE_POWER, horizontalDistance);
        rotateBot(0.3, rotate);
        moveBotDiag(DRIVE_MOVE_POWER, CryptoboxDistance, direction);

        placeGlyph();

        // position bot for teleop
        final int safeZoneDistance = -700;
        moveBotDiag(DRIVE_MOVE_POWER, safeZoneDistance, direction);
    }

    // move from balancing stone to and rotate to position the glyph in the center column
    private void placeGlyphInCenterColumnAndParkInSafeZone() {
        final double DRIVE_MOVE_POWER = 0.4;
        final int CryptoboxDistance = 1250;
        final int horizontalDistance = 900;
        final int rotate = DRIVE_ROTATE90_DISTANCE * 3 / 2;
        final int direction = 1;

        moveBotSide(DRIVE_MOVE_POWER, horizontalDistance);
        rotateBot(0.3, rotate);
        moveBotDiag(DRIVE_MOVE_POWER, CryptoboxDistance, direction);

        placeGlyph();

        // position bot for teleop
        final int safeZoneDistance = -1600;
        moveBotDiag(DRIVE_MOVE_POWER, safeZoneDistance, direction);
    }

    private void rotateBot(double power, int distance) {
        telemetry.addLine("Rotate Bot " + power + " " + distance);
        telemetry.update();

        driveUpMotor.setTargetPosition(driveUpMotor.getCurrentPosition() + (distance));
        driveUpMotor.setPower(power);

        driveDownMotor.setTargetPosition(driveDownMotor.getCurrentPosition() + (distance));
        driveDownMotor.setPower(power);

        driveLeftMotor.setTargetPosition(driveLeftMotor.getCurrentPosition() + (distance));
        driveLeftMotor.setPower(power);

        driveRightMotor.setTargetPosition(driveRightMotor.getCurrentPosition() + (distance));
        driveRightMotor.setPower(power);

        while (driveUpMotor.isBusy()) {
            idle();
        }
    }

    private void moveBotSide(double power, int distance) {
        telemetry.addLine("Move Bot" + power + distance);
        telemetry.update();

        driveUpMotor.setTargetPosition(driveUpMotor.getCurrentPosition() + distance);
        driveUpMotor.setPower(power);

        driveDownMotor.setTargetPosition(driveDownMotor.getCurrentPosition() - distance);
        driveDownMotor.setPower(-power);

        driveLeftMotor.setTargetPosition(driveLeftMotor.getCurrentPosition() + distance);
        driveLeftMotor.setPower(power);

        driveRightMotor.setTargetPosition(driveRightMotor.getCurrentPosition() - distance);
        driveRightMotor.setPower(-power);

        while (driveUpMotor.isBusy()) {
            idle();
        }
    }

    private void moveBotDiag(double power, int distance, int direction) {
        telemetry.addLine("Move Bot Diagonal " + power + distance);
        telemetry.update();

        driveLeftMotor.setTargetPosition(driveLeftMotor.getCurrentPosition() - direction * distance);
        driveLeftMotor.setPower(power);

        driveRightMotor.setTargetPosition(driveRightMotor.getCurrentPosition() + direction * distance);
        driveRightMotor.setPower(-power);
    }

    private void wait(double length) {
        telemetry.addLine("Waiting...");
        telemetry.update();

        double waitUntil = time + length;
        while (time < waitUntil) {
            idle();
        }
    }

    // turned visualController into VuMark sensor
    private RelicRecoveryVuMark senseRecoveryVuMark() throws InterruptedException {
        if (pictographIsUknown) {
            visualC.look();
            pictographIsUknown = false;
        }
        return visualC.pictograph;
    }

    // turned visualController into jewel color sensor
    private VisualController.JewelColor senseLeftJewelColor() throws InterruptedException {
        if (pictographIsUknown) {
            visualC.look();
            pictographIsUknown = false;
        }
        return visualC.rightJewel;
    }

}