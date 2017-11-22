package org.firstinspires.ftc.teamcode.MainBot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import static org.firstinspires.ftc.teamcode.MainBot.autonomous.AutonomousController.*;

@Autonomous(name = "Autonomous Controller Test", group = "MainBot")
public class AutonomousTest extends LinearOpMode {

    private static VisualController.JewelColor TEAM_COLOR = VisualController.JewelColor.RED;
    private AutonomousController a = new AutonomousController();
    private VisualController v = new VisualController();

    private static int ENC_90 = 1523;

    private double waitUntil = 0;
    private int state = 0;


    private int[][] targets = new int[][]{
           //L, C, R
            //shift
            {-150, 0, 0},
            //knock
            {250, 0, -250},
            //toCrypto
            {2550, 1900, 3300},
            //rotCrypto
            {ENC_90*3/2, ENC_90*3/2, ENC_90/2},
            //push (1 = UD, 0 = LR)
            {1, 1, 0},
            {-800, -800, -800},
            //rotate
            {0, 0, 0}
    };












    @Override
    public void runOpMode() throws InterruptedException {
        a.init(telemetry, hardwareMap);

        telemetry.addLine("Status: Initialized and ready!");
        telemetry.update();

        waitForStart();

        a.close();
        wait(0.6);

        a.lift();
        waitFor(GLYPH);

        a.moveBot(targets[0][0]);
        waitFor(UP);

        v.look();

        state = (v.leftJewel == TEAM_COLOR ? 0 : 2);

        a.lowerJArm();
        waitFor(JEWEL);

        a.rotateBot(targets[1][state], 0.7);
        waitFor(UP);

        a.raiseJArm();

        a.rotateBot(-targets[1][state], 0.7);
        waitFor(UP);

        state = (v.pictograph == RelicRecoveryVuMark.LEFT ? 0 : (v.pictograph == RelicRecoveryVuMark.CENTER ? 1 : 2));

        a.moveBot(targets[2][state]);
        waitFor(UP);

        a.rotateBot(targets[3][state], 0.7);
        waitFor(UP);

        if (targets[4][state] == 1) {
            a.moveBotDiUD(targets[5][state]);
            waitFor(UP);
        } else {
            a.moveBotDiLR(targets[5][state]);
            waitFor(LEFT);
        }


        a.lower();
        waitFor(GLYPH);

        a.open();

        if (targets[4][state] == 1) {
            a.moveBotDiUD(-targets[5][state]);
            waitFor(UP);
        } else {
            a.moveBotDiLR(-targets[5][state]);
            waitFor(LEFT);
        }

        a.rotateBot(targets[6][state]);
        waitFor(UP);
    }

    private void waitFor(int motor) {
        while (a.motors[motor].isBusy())
            idle();
        a.reset();
    }

    private void wait(double s) {
        waitUntil = time + s;
        while (time < waitUntil)
            idle();
        a.reset();
    }
}