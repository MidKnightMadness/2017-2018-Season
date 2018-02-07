package org.firstinspires.ftc.teamcode.MainBot.autonomous.Tests.OlderTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.MainBot.autonomous.VisualController;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

@Disabled
@Autonomous(name = "BlueNonRecovery", group = "Main Bot")
public class BlueNonRecovery extends LinearOpMode {
    private static VisualController.JewelColor TEAM_COLOR = VisualController.JewelColor.BLUE;
    private static double JEWEL_ARM_POWER = 0.3;
    private static int JEWEL_ARM_DISTANCE = 600;
    private static double DRIVE_ROTATE_POWER = -0.3;
    private static int DRIVE_ROTATE_DISTANCE = 200;
    private static double DRIVE_MOVE_POWER = 0.4;
    private static int DRIVE_MOVE_DISTANCE = -1900;
    private static int DRIVE_MOVE_SIDE_DISTANCE = 825;

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

        visualC.look();
        lowerArm();
        rotateBot(true);
        raiseArm();
        rotateBot(false);
        moveBot();
        if (visualC.pictograph == RelicRecoveryVuMark.RIGHT){
            DRIVE_MOVE_SIDE_DISTANCE = DRIVE_MOVE_SIDE_DISTANCE + 650;
        }
        else if (visualC.pictograph == RelicRecoveryVuMark.LEFT){
            DRIVE_MOVE_SIDE_DISTANCE = DRIVE_MOVE_SIDE_DISTANCE - 650;
        }
        moveBotSideways();
    }// -650, 0, 650


    void initialize(HardwareMap hardwareMap) {
        jewelMotor = hardwareMap.dcMotor.get(CrossCommunicator.Jewel.MOTOR);
        jewelMotor.resetDeviceConfigurationForOpMode();
        jewelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveUpMotor = hardwareMap.dcMotor.get(CrossCommunicator.Drive.FRONT_LEFT);
        driveUpMotor.resetDeviceConfigurationForOpMode();
        driveUpMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveUpMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveDownMotor = hardwareMap.dcMotor.get(CrossCommunicator.Drive.BACK_RIGHT);
        driveDownMotor.resetDeviceConfigurationForOpMode();
        driveDownMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveDownMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveLeftMotor = hardwareMap.dcMotor.get(CrossCommunicator.Drive.BACK_LEFT);
        driveLeftMotor.resetDeviceConfigurationForOpMode();
        driveLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveRightMotor = hardwareMap.dcMotor.get(CrossCommunicator.Drive.FRONT_RIGHT);
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

    void rotateBot(Boolean reset) {
        telemetry.addLine("Rotate Bot " + (reset ? "First..." : "Second..."));
        telemetry.update();

        int neg = (visualC.rightJewel == TEAM_COLOR) ? 1 : -1;
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
    void moveBotSideways() {
        telemetry.addLine("Move Bot...");
        telemetry.update();

        driveUpMotor.setTargetPosition(driveUpMotor.getCurrentPosition() + DRIVE_MOVE_SIDE_DISTANCE);
        driveUpMotor.setPower(DRIVE_MOVE_POWER);

        driveDownMotor.setTargetPosition(driveDownMotor.getCurrentPosition() - DRIVE_MOVE_SIDE_DISTANCE);
        driveDownMotor.setPower(-DRIVE_MOVE_POWER);

        driveLeftMotor.setTargetPosition(driveLeftMotor.getCurrentPosition() - DRIVE_MOVE_SIDE_DISTANCE);
        driveLeftMotor.setPower(-DRIVE_MOVE_POWER);

        driveRightMotor.setTargetPosition(driveRightMotor.getCurrentPosition() + DRIVE_MOVE_SIDE_DISTANCE);
        driveRightMotor.setPower(DRIVE_MOVE_POWER);

        while (driveUpMotor.isBusy()) {
            idle();
        }
    }
}