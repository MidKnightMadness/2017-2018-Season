package org.firstinspires.ftc.teamcode.MainBot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import static org.firstinspires.ftc.teamcode.MainBot.autonomous.AutonomousController.*;

@Autonomous(name = "Red Recovery", group = "MainBot")
public class RedRecovery extends LinearOpMode {

    private static VisualController.JewelColor TEAM_COLOR = VisualController.JewelColor.RED;
    private AutonomousController a = new AutonomousController();
    private VisualController v = new VisualController();

    private static int ENC_90 = 1575;

    private double waitUntil = 0;
    private int state = 0;


    private int[][] targets = new int[][]{
            //L, C, R
            //shift
            {-100, 0, 0}, // shift because the robot starts off-center
            //knock
            {250, 0, -250}, // amount to rotate to knock jewel
            //toCrypto
            {2500, 1800, 2900}, // amount to move sideways to get to the cryptobox
            //rotCrypto
            {ENC_90 * 3 / 2, ENC_90 * 3 / 2, ENC_90 / 2}, // amount to rotate to align to put glyph in cryptobox
            //push (1 = LR, 0 = UD)
            {1, 1, 0}, // the diagonal boolean to show the direction that the robot is facing. LR or UD?
            {-700, -700, -700}, // Move towards cryptobox before releasing
            {-1100, -1100, -1100}, // Move into cryptobox after releasing
            {1000, 1000, 1000}, // Move away from cryptobox
            //rotate
            {ENC_90 * 3 / 2, ENC_90 * 3 / 2, ENC_90 * 5 / 2}, // amount to rotate at the end.
            {-2000, -2000, -2000}
    };












    @Override
    public void runOpMode() throws InterruptedException {
        a.init(telemetry, hardwareMap, v);
        v.init(telemetry, hardwareMap);

        v.saveTeamColor(0);

        telemetry.addLine("Status: Initialized and ready!");
        telemetry.update();

        waitForStart();

        a.close();
        waitUntil = time + 1d;

        a.look();

        while (time < waitUntil)
            idle();

        a.lift();
        waitFor(ELEV);



        a.moveBot(targets[0][0]);
        waitFor(UP);

        if (v.rightJewel != null) {
            state = (v.rightJewel == TEAM_COLOR ? 0 : 2);

            a.lowerJArm();
            wait(1d);

            a.rotateBot(targets[1][state], 0.7);
            waitFor(UP);

            a.raiseJArm();
            wait(1d);

            a.rotateBot(-targets[1][state], 0.7);
            waitFor(UP);
        }

        state = (v.pictograph == RelicRecoveryVuMark.LEFT ? 0 : (v.pictograph == RelicRecoveryVuMark.CENTER ? 1 : 2));

        a.moveBot(targets[2][state]);
        waitFor(UP);

        a.rotateBot(targets[3][state], 0.7);
        waitFor(UP);

        if (targets[4][state] == 0) {
            a.moveBotDiUD(targets[5][state]);
            waitFor(UP);
        } else {
            a.moveBotDiLR(targets[5][state]);
            waitFor(LEFT);
        }




        a.lower();
        waitFor(ELEV);

        a.open();

        if (targets[4][state] == 0) {
            a.moveBotDiUD(targets[6][state]);
            waitFor(UP);
        } else {
            a.moveBotDiLR(targets[6][state]);
            waitFor(LEFT);
        }

        if (targets[4][state] == 0) {
            a.moveBotDiUD(targets[7][state]);
            waitFor(UP);
        } else {
            a.moveBotDiLR(targets[7][state]);
            waitFor(LEFT);
        }

        a.grabStop();

        a.rotateBot(targets[8][state]);
        waitFor(UP);

        //Second Glyph

        a.moveBot(targets[9][state]);
        waitFor(UP);

        a.close();
        wait(1d);

        a.lift(2000);
        waitFor(ELEV);

        a.moveBot(-targets[9][state] - 300);
        waitFor(UP);

        a.rotateBot(-targets[8][state]);
        waitFor(UP);

        if (targets[4][state] == 0) {
            a.moveBotDiUD(targets[6][state] - 800);
            waitFor(UP);
        } else {
            a.moveBotDiLR(targets[6][state] - 800);
            waitFor(LEFT);
        }

        a.lower(1000);
        waitFor(ELEV);

        a.open();
        wait(0.5d);

        if (targets[4][state] == 0) {
            a.moveBotDiUD(targets[7][state]);
            waitFor(UP);
        } else {
            a.moveBotDiLR(targets[7][state]);
            waitFor(LEFT);
        }

        a.lower(1000);

        a.rotateBot(targets[8][state]);
        waitFor(UP);

        //Dance
    }

    private void waitFor(int motor) {
        while (a.motors[motor].isBusy()) {
            telemetry.addData("motor", a.motors[motor].getPower());
            //a.updateMotors();
            idle();
        }
        a.reset();
    }

    private void wait(double s) {
        waitUntil = time + s;
        while (time < waitUntil) {
            //a.updateMotors();
            idle();
        }
        a.reset();
    }
}