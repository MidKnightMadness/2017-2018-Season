package org.firstinspires.ftc.teamcode.MainBot.autonomous.SimpleBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MainBot.autonomous.VisualAssembly.VisualController;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

@Autonomous(name = "MainBot", group = "Main Bot")
public class SimpleBot extends LinearOpMode {
    private static VisualController.JewelColor TEAM_COLOR = VisualController.JewelColor.BLUE;
    private static double JEWEL_ARM_POWER = 0.3;
    private static int JEWEL_ARM_DISTANCE = 529;
    private static double DRIVE_ROTATE_POWER = 0.2;
    private static int DRIVE_ROTATE_DISTANCE = 50;
    private static double DRIVE_MOVE_POWER = 0.2;
    private static int DRIVE_MOVE_DISTANCE = 4000;
    private DcMotor jewelMotor;
    private DcMotor driveUpMotor;
    private DcMotor driveDownMotor;
    private DcMotor driveLeftMotor;
    private DcMotor driveRightMotor;
    private VisualController visualC = new VisualController();

    void lowerArm() {
        jewelMotor.setTargetPosition(jewelMotor.getCurrentPosition() - JEWEL_ARM_DISTANCE);
        jewelMotor.setPower(JEWEL_ARM_POWER);
        while (jewelMotor.isBusy()) {
            idle();
        }
    }

    void raiseArm() {
        jewelMotor.setTargetPosition(jewelMotor.getCurrentPosition() + JEWEL_ARM_DISTANCE);
        jewelMotor.setPower(-JEWEL_ARM_POWER);
        while (jewelMotor.isBusy()) {
            idle();
        }
    }


    void rotateBot(Boolean reset) {
        int neg = (visualC.leftJewel == TEAM_COLOR) ? 1 : -1;
        neg = (reset) ? neg : -neg;

        driveUpMotor.setTargetPosition(neg * DRIVE_ROTATE_DISTANCE);
        driveUpMotor.setPower(neg * DRIVE_ROTATE_POWER);

        driveDownMotor.setTargetPosition(neg * DRIVE_ROTATE_DISTANCE);
        driveDownMotor.setPower(neg * DRIVE_ROTATE_POWER);

        driveLeftMotor.setTargetPosition(neg * DRIVE_ROTATE_DISTANCE);
        driveLeftMotor.setPower(neg * DRIVE_ROTATE_POWER);

        driveRightMotor.setTargetPosition(neg * DRIVE_ROTATE_DISTANCE);
        driveRightMotor.setPower(neg * DRIVE_ROTATE_POWER);

        while (driveUpMotor.isBusy()) {
            idle();
        }

    }

    void moveBot() {
        driveUpMotor.setTargetPosition(-DRIVE_MOVE_DISTANCE);
        driveUpMotor.setPower(-DRIVE_MOVE_POWER);

        driveDownMotor.setTargetPosition(-DRIVE_MOVE_DISTANCE);
        driveDownMotor.setPower(-DRIVE_MOVE_POWER);

        driveLeftMotor.setTargetPosition(DRIVE_MOVE_DISTANCE);
        driveLeftMotor.setPower(DRIVE_MOVE_POWER);

        driveRightMotor.setTargetPosition(DRIVE_MOVE_DISTANCE);
        driveRightMotor.setPower(DRIVE_MOVE_POWER);

        while (driveUpMotor.isBusy()) {
            idle();
        }

    }


    @Override
    public void runOpMode() throws InterruptedException {

        jewelMotor = hardwareMap.dcMotor.get(CrossCommunicator.Jewel.MOTOR);
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


        visualC.init(telemetry, hardwareMap);

        telemetry.addLine("Ready to go!");
        telemetry.update();


        waitForStart();

        visualC.look();
        lowerArm();
        rotateBot(true);
        raiseArm();
        rotateBot(false);
        moveBot();
    }
}