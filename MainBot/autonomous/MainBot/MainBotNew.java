package org.firstinspires.ftc.teamcode.MainBot.autonomous.MainBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MainBot.autonomous.JewelAssembly.JewelAssemblyAutonomousController;
import org.firstinspires.ftc.teamcode.MainBot.autonomous.VisualAssembly.VisualController;
import org.firstinspires.ftc.teamcode.MainBot.teleop.DriveAssembly.Tests.DriveAssemblyNotSoOldController;

@Autonomous(name = "MainBotNew", group = "Main Bot")
public class MainBotNew extends LinearOpMode {
    private static VisualController.JewelColor TEAM_COLOR = VisualController.JewelColor.BLUE;
    @Override
    public void runOpMode() throws InterruptedException {

        DriveAssemblyNotSoOldController driveC = new DriveAssemblyNotSoOldController();
        JewelAssemblyAutonomousController jewelC = new JewelAssemblyAutonomousController();
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

        visualC.look(false);
        jewelC.down();
        while (jewelC.isBusy()) {
            idle();
        }

        driveC.setTarget(0.1, 0, 0, (visualC.leftJewel == TEAM_COLOR ? 100 : 80), 1, false);
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

        driveC.setTarget(0.5, 0, 0, (visualC.leftJewel == TEAM_COLOR ? 100 : 80), 1, false);
        while (!driveC.reachedTargetRotation) {
            driveC.update();
            telemetry.update();
            idle();
        }
    }
}