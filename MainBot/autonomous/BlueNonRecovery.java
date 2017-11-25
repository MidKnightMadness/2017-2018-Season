package org.firstinspires.ftc.teamcode.MainBot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import static org.firstinspires.ftc.teamcode.MainBot.autonomous.AutonomousController.GLYPH;
import static org.firstinspires.ftc.teamcode.MainBot.autonomous.AutonomousController.JEWEL;
import static org.firstinspires.ftc.teamcode.MainBot.autonomous.AutonomousController.LEFT;
import static org.firstinspires.ftc.teamcode.MainBot.autonomous.AutonomousController.UP;

@Autonomous(name = "Blue Non-Recovery", group = "MainBot")
public class BlueNonRecovery extends LinearOpMode {

    private static VisualController.JewelColor TEAM_COLOR = VisualController.JewelColor.BLUE;
    private AutonomousController a = new AutonomousController();
    private VisualController v = new VisualController();

    private static int ENC_90 = 1575;

    private double waitUntil = 0;
    private int state = 0;


    private int[][] targets = new int[][]{
            //L, C, R
            //shift
            {-150, 0, 0},
            //knock
            {250, 0, -250},
            //toCrypto
            {-1700, -1700, -1700},
            //rotate
            {ENC_90, ENC_90, ENC_90},
            //toCrypto
            {1100, 1400, 2000},
            //rotCrypto
            {-ENC_90/2, -ENC_90/2, -ENC_90/2},
            //push (1 = UD, 0 = LR)
            {1, 1, 1},
            {-500, -500, -500},
            {-1300, -1300, -1300},
            {700, 700, 700},
            //rotate
            {-ENC_90*3/2, -ENC_90*3/2, -ENC_90*3/2},
            {0, 0, 600}
    };












    @Override
    public void runOpMode() throws InterruptedException {
        a.init(telemetry, hardwareMap);
        v.init(telemetry, hardwareMap);

        v.saveTeamColor(3);

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
        wait(1d);

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

        a.moveBot(targets[4][state]);
        waitFor(UP);

        a.rotateBot(targets[5][state], 0.7);
        waitFor(UP);





        if (targets[6][state] == 0) {
            a.moveBotDiUD(targets[7][state]);
            waitFor(UP);
        } else {
            a.moveBotDiLR(targets[7][state]);
            waitFor(LEFT);
        }




        a.lower();
        waitFor(GLYPH);

        a.open();

        if (targets[6][state] == 0) {
            a.moveBotDiUD(targets[8][state]);
            waitFor(UP);
        } else {
            a.moveBotDiLR(targets[8][state]);
            waitFor(LEFT);
        }

        if (targets[6][state] == 0) {
            a.moveBotDiUD(targets[9][state]);
            waitFor(UP);
        } else {
            a.moveBotDiLR(targets[9][state]);
            waitFor(LEFT);
        }

        a.rotateBot(targets[10][state]);
        waitFor(UP);

        a.moveBot(targets[11][state]);
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