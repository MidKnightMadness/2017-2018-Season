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
            {-75, 0, 75},
            //knock
            {250, 0, -250},
            //toCrypto
            {2550, 1900, 3300},
            //rotCrypto
            {ENC_90/2, ENC_90/2, -ENC_90/2},
            //push
            {-800, -800, -800},
            //reverse
            {400, 400, 400},
            //rotate
            {0, 0, ENC_90 * 2/3}
    };












    @Override
    public void runOpMode() throws InterruptedException {
        a.init(telemetry, hardwareMap);

        telemetry.addLine("Status: Initialized and ready!");
        telemetry.update();

        waitForStart();

        a.close();
        wait(0.6);

        a.moveBot(targets[0][0]);
        waitFor(UP);

        v.look();

        state = (v.leftJewel == TEAM_COLOR ? 0 : 2);

        a.lowerJArm();
        waitFor(JEWEL);

        a.rotateBot(targets[1][state]);
        waitFor(UP);

        a.raiseJArm();
        waitFor(JEWEL);

        a.rotateBot(-targets[1][state]);
        waitFor(UP);

        state = (v.pictograph == RelicRecoveryVuMark.LEFT ? 0 : (v.pictograph == RelicRecoveryVuMark.CENTER ? 1 : 2));

        a.moveBot(targets[2][state]);
        waitFor(UP);

        a.rotateBot(targets[3][state], 0.7);
        waitFor(UP);

        a.moveBot(targets[4][state]);
        waitFor(UP);

        a.lower();
        waitFor(GLYPH);

        a.open();

        a.moveBot(targets[5][state]);
        waitFor(UP);

        a.rotateBot(targets[6][state]);
    }

    private void waitFor(int motor) {
        while (a.motors[motor].isBusy())
            idle();
    }

    private void wait(double s) {
        waitUntil = time + s;
        while (time < waitUntil)
            idle();
    }
}