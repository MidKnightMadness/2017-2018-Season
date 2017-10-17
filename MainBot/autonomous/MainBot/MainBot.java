package org.firstinspires.ftc.teamcode.MainBot.autonomous.MainBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MainBot.autonomous.Vuforia.VisualController;
import org.firstinspires.ftc.teamcode.MainBot.teleop.DriveAssembly.DriveAssemblyController;
import org.firstinspires.ftc.teamcode.MainBot.teleop.GlyphAssembly.GlyphAssemblyController;
import org.firstinspires.ftc.teamcode.MainBot.teleop.JewelAssembly.JewelAssemblyController;

@Autonomous(name = "MainBot", group = "Main Bot")
public class MainBot extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DriveAssemblyController driveC = new DriveAssemblyController();
        JewelAssemblyController jewelC = new JewelAssemblyController();
        GlyphAssemblyController glyphC = new GlyphAssemblyController();
        VisualController visualC = new VisualController();

        driveC.init(telemetry, hardwareMap);
        jewelC.init(telemetry, hardwareMap);
        glyphC.init(telemetry, hardwareMap);
        visualC.init(telemetry, hardwareMap);

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();

        visualC.look(false);

        /*glyphC.grab();
        jewelC.down();
        driveC.move(0.2, 0, 0, 10);

        while (!(driveC.rotated && jewelC.reachedTarget && glyphC.reachedTarget)) {
            driveC.update();
            jewelC.update();
            glyphC.update();
        }

        jewelC.up();
        */

        // Do something useful

    }
}