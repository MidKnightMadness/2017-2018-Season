package org.firstinspires.ftc.teamcode.MainBot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.MainBot.autonomous.AutonomousController.*;

@TeleOp(name = "Autonomous Teleop", group = "MainBot")
public class AutonomousTeleop extends LinearOpMode {

    private AutonomousController a = new AutonomousController();

    private static int ENC_90 = 1523;

    private int amount = 0;
    private double waitUntil = 0;
    private int p = 0;
    private int state = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        a.init(telemetry, hardwareMap);
        waitForStart();

        a.close();
        wait(0.6);

        a.lift();
        waitFor(GLYPH);
        a.reset();

        while (!gamepad1.x && opModeIsActive()) {
            if (Math.abs(gamepad1.left_stick_x) > 0.2) {
                amount += (int)Math.signum(gamepad1.left_stick_x)*50;
                a.moveBot((int)Math.signum(gamepad1.left_stick_x)*50);
            }
            telemetry.addData("StartMove: ", amount);
            telemetry.addData("StartMove: ", gamepad1.left_stick_x);
            telemetry.update();
            waitFor(UP);
        }

        while (gamepad1.x)
            idle();
        a.reset();
        amount = 0;

        while (!gamepad1.x && opModeIsActive()) {
            if (Math.abs(gamepad1.left_stick_x) > 0.2) {
                amount += (int)Math.signum(gamepad1.left_stick_x)*50;
                a.rotateBot((int)Math.signum(gamepad1.left_stick_x)*50);
            }
            telemetry.addData("UP: ", amount);
            telemetry.update();
            waitFor(UP);
        }

        p = amount;

        while (gamepad1.x)
            idle();
        a.reset();
        amount = 0;

        a.lowerJArm();
        waitFor(JEWEL);
        a.reset();

        a.rotateBot(-p);
        waitFor(UP);
        a.reset();

        a.raiseJArm();
        waitFor(JEWEL);
        a.reset();

        while (!gamepad1.x && opModeIsActive()) {
            if (Math.abs(gamepad1.left_stick_x) > 0.2) {
                amount += (int)Math.signum(gamepad1.left_stick_x)*50;
                a.moveBot((int)Math.signum(gamepad1.left_stick_x)*50);
            }
            telemetry.addData("UP: ", amount);
            telemetry.update();
            waitFor(UP);
        }

        while (gamepad1.x)
            idle();
        a.reset();
        amount = 0;

        while (!gamepad1.x && opModeIsActive()) {
            if (Math.abs(gamepad1.left_stick_x) > 0.2) {
                amount += (int)Math.signum(gamepad1.left_stick_x)*50;
                a.rotateBot((int)Math.signum(gamepad1.left_stick_x)*50);
            }
            telemetry.addData("UP: ", amount);
            telemetry.update();
            waitFor(UP);
        }

        while (gamepad1.x)
            idle();
        a.reset();
        amount = 0;

        while ((Math.signum(gamepad1.left_stick_x)< 0.2) && (Math.signum(gamepad1.left_stick_y)< 0.2))
            idle();
        boolean s = (Math.signum(gamepad1.left_stick_x)< 0.2);
        if (s) {
            while (!gamepad1.x && opModeIsActive()) {
                if (Math.abs(gamepad1.left_stick_x) > 0.2) {
                amount += (int) Math.signum(gamepad1.left_stick_x) * 50;
                    a.moveBotDiUD((int) Math.signum(gamepad1.left_stick_x) * 50);
                }
                telemetry.addData("UP: ", amount);
                telemetry.update();
                waitFor(UP);
            }
            p = a.getPos(UP);
        } else {
            while (!gamepad1.x && opModeIsActive()) {
                if (Math.abs(gamepad1.left_stick_y) > 0.2) {
                amount += (int) Math.signum(gamepad1.left_stick_y) * 50;
                    a.moveBotDiLR((int) Math.signum(gamepad1.left_stick_y) * 50);
                }
                telemetry.addData("UP: ", a.getPos(LEFT));
                telemetry.update();
                waitFor(LEFT);
            }
            p = a.getPos(LEFT);
        }

        while (gamepad1.x)
            idle();
        a.reset();
        amount = 0;

        a.lower();
        waitFor(GLYPH);
        a.reset();

        a.open();

        if (s) {
            a.moveBotDiUD(p);
            waitFor(UP);
        } else {
            a.moveBotDiLR(p);
            waitFor(LEFT);
        }
        a.reset();

        while (!gamepad1.x && opModeIsActive()) {
            if (Math.abs(gamepad1.left_stick_x) > 0.2) {
                amount += (int)Math.signum(gamepad1.left_stick_x)*50;
                a.rotateBot((int)Math.signum(gamepad1.left_stick_x)*50);
            }
            telemetry.addData("UP: ", amount);
            telemetry.update();
            waitFor(UP);
        }

        while (gamepad1.x)
            idle();
        a.reset();
        amount = 0;
    }

    private void waitFor(int motor) {
        while (a.motors[motor].isBusy()) {
            telemetry.addData("Amount" + motor + ": ", amount);
            telemetry.update();
            idle();
        }
        idle();
    }

    private void wait(double s) {
        waitUntil = time + s;
        while (time < waitUntil)
            idle();
        a.reset();
    }
}