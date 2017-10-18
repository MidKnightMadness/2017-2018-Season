package org.firstinspires.ftc.teamcode.MainBot.autonomous.MainBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MainBot.autonomous.Vuforia.VisualController;
import org.firstinspires.ftc.teamcode.MainBot.teleop.DriveAssembly.DriveAssemblyController;
import org.firstinspires.ftc.teamcode.MainBot.teleop.GlyphAssembly.GlyphAssemblyController;
import org.firstinspires.ftc.teamcode.MainBot.teleop.JewelAssembly.JewelAssemblyController;
import org.firstinspires.ftc.teamcode.MainBot.teleop.JewelAssembly.JewelAssemblyNewController;

@Autonomous(name = "MainBot", group = "Main Bot")
public class MainBot extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DriveAssemblyController driveC = new DriveAssemblyController();
        JewelAssemblyNewController jewelC = new JewelAssemblyNewController();
        GlyphAssemblyController glyphC = new GlyphAssemblyController();
        VisualController visualC = new VisualController();
        driveC.init(telemetry, hardwareMap);
        jewelC.init(telemetry, hardwareMap);
        glyphC.init(telemetry, hardwareMap);
        visualC.init(telemetry, hardwareMap);

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();
        driveC.start();

        visualC.look(false);
        jewelC.down();
        driveC.setTarget(0.1, 0, 0, (visualC.leftJewel == VisualController.JewelColor.BLUE ? 10 : -10), (visualC.leftJewel == VisualController.JewelColor.BLUE ? 1 : -1), false);
        while (!driveC.reachedTargetRotation || jewelC.isBusy()) {
            driveC.update();
            jewelC.update();
            idle();
        }

        jewelC.up();
        driveC.setTarget(0.1, 0, 0, (visualC.leftJewel == VisualController.JewelColor.BLUE ? -10 : 10), (visualC.leftJewel == VisualController.JewelColor.BLUE ? -1 : 1), false);
        while (!driveC.reachedTargetRotation || jewelC.isBusy()) {
            driveC.update();
            jewelC.update();
            idle();
        }

        // Do something useful
    }
}