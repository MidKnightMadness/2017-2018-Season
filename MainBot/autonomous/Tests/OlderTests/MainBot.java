package org.firstinspires.ftc.teamcode.MainBot.autonomous.Tests.OlderTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MainBot.autonomous.VisualController;
import org.firstinspires.ftc.teamcode.MainBot.teleop.DriveAssembly.Tests.DriveAssemblyNotSoOldController;

@Disabled
@Autonomous(name = "MainBot", group = "Main Bot")
public class MainBot extends LinearOpMode {
    private static VisualController.JewelColor TEAM_COLOR = VisualController.JewelColor.BLUE;
    @Override
    public void runOpMode() throws InterruptedException {

        DriveAssemblyNotSoOldController driveC = new DriveAssemblyNotSoOldController();
        JewelController jewelC = new JewelController();
        //GlyphAssemblyController glyphC = new GlyphAssemblyController();
        VisualController visualC = new VisualController();
        driveC.init(telemetry, hardwareMap);
        jewelC.init(telemetry, hardwareMap);
        //glyphC.init(telemetry, hardwareMap);
        visualC.init(telemetry, hardwareMap);

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();
        driveC.start();

        visualC.look();
        jewelC.down();
        while (jewelC.isBusy()) {
            idle();
        }

        driveC.setTarget(0.1, 0, 0, (visualC.rightJewel == TEAM_COLOR ? 100 : 80), 1, false);
        while (!driveC.reachedTargetRotation) {
            driveC.update();
            telemetry.update();
            idle();
        }

        jewelC.up();
        while (jewelC.isBusy()) {
            idle();
        }

        driveC.setTarget(0.5, 3828.5, 180, 0, 0, false);
        while (!driveC.reachedTargetTranslation) {
            driveC.update();
            telemetry.update();
            idle();
        }

        driveC.setTarget(0.5, 0, 0, (visualC.rightJewel == TEAM_COLOR ? 100 : 80), 1, false);
        while (!driveC.reachedTargetRotation) {
            driveC.update();
            telemetry.update();
            idle();
        }
    }
}