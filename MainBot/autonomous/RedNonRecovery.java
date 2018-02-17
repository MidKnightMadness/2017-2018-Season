package org.firstinspires.ftc.teamcode.MainBot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import static org.firstinspires.ftc.teamcode.MainBot.autonomous.AutonomousController.ELEV;
import static org.firstinspires.ftc.teamcode.MainBot.autonomous.AutonomousController.BACK_LEFT;
import static org.firstinspires.ftc.teamcode.MainBot.autonomous.AutonomousController.FRONT_LEFT;

@Autonomous(name = "Red Non-Recovery", group = "MainBot")
public class RedNonRecovery extends LinearOpMode {

    private static VisualController.JewelColor TEAM_COLOR = VisualController.JewelColor.RED;
    private AutonomousController a = new AutonomousController();
    private VisualController v = new VisualController();

    private static int ENC_90 = 1595;

    private double waitUntil = 0;
    private int state = 0;


    private int[][] targets = new int[][]{
           //L, C, R
            //shift
            {-100, 0, 0},
            //knock
            {250, 0, -250},
            //toCrypto
            {1800, 1800, 1800},
            //rotate
            {ENC_90, ENC_90, ENC_90},
            //toCrypto
            {2300, 1750, 1200},
            //rotCrypto
            {ENC_90/2, ENC_90/2, ENC_90/2},
            //push (1 = LR, 0 = UD)
            {0, 0, 0},
            {-600, -600, -600},
            {-1100, -1100, -1100},
            {900, 900, 900},
            //rotate
            {ENC_90*3/2, ENC_90*3/2, ENC_90*3/2},
            {600, 0, 0}
    };












    @Override
    public void runOpMode() throws InterruptedException {
        a.init(telemetry, hardwareMap, v);
        v.init(telemetry, hardwareMap);

        v.saveTeamColor(1);

        telemetry.addLine("Status: Initialized and ready!");
        telemetry.update();

        waitForStart();

        a.close();
        wait(1d);

        a.lift();
        waitFor(ELEV);

        a.look();

        a.moveBot(targets[0][0]);
        waitFor(FRONT_LEFT);

        if (v.rightJewel != null) {
            state = (v.rightJewel == TEAM_COLOR ? 0 : 2);

            a.lowerJArm();
            wait(1d);

            a.rotateBot(targets[1][state], 0.7);
            waitFor(FRONT_LEFT);

            a.raiseJArm();
            wait(1d);

            a.rotateBot(-targets[1][state], 0.7);
            waitFor(FRONT_LEFT);
        }

        state = (v.pictograph == RelicRecoveryVuMark.LEFT ? 0 : (v.pictograph == RelicRecoveryVuMark.CENTER ? 1 : 2));

        a.moveBot(targets[2][state]);
        waitFor(FRONT_LEFT);

        a.rotateBot(targets[3][state], 0.7);
        waitFor(FRONT_LEFT);

        a.moveBot(targets[4][state]);
        waitFor(FRONT_LEFT);

        a.rotateBot(targets[5][state], 0.7);
        waitFor(FRONT_LEFT);





        if (targets[6][state] == 0) {
            a.moveBotDiUD(targets[7][state]);
            waitFor(FRONT_LEFT);
        } else {
            a.moveBotDiLR(targets[7][state]);
            waitFor(BACK_LEFT);
        }




        a.lower();
        waitFor(ELEV);

        a.open();

        if (targets[6][state] == 0) {
            a.moveBotDiUD(targets[8][state]);
            waitFor(FRONT_LEFT);
        } else {
            a.moveBotDiLR(targets[8][state]);
            waitFor(BACK_LEFT);
        }

        if (targets[6][state] == 0) {
            a.moveBotDiUD(targets[9][state]);
            waitFor(FRONT_LEFT);
        } else {
            a.moveBotDiLR(targets[9][state]);
            waitFor(BACK_LEFT);
        }

        a.grabStop();

        a.rotateBot(targets[10][state]);
        waitFor(FRONT_LEFT);

        a.moveBot(targets[11][state]);
        waitFor(FRONT_LEFT);
    }

    private void waitFor(int motor) throws InterruptedException {
        while (a.motors[motor].isBusy())
            idle();
        a.reset();
    }

    private void wait(double s) throws InterruptedException {
        waitUntil = time + s;
        while (time < waitUntil)
            idle();
        a.reset();
    }
}