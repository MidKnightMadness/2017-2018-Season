package org.firstinspires.ftc.teamcode.MainBot.autonomous.Tests.Visual;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Disabled
@Autonomous(name = "aVuforia: VuMark View", group = "Main Bot")
public class VuMarkView extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //Init here
        telemetry.addLine("Initializing Vuforia");
        telemetry.update();

        //init the VuforiaLocalizer parameters object
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);

        //Which side of the phone?
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //License Key
        params.vuforiaLicenseKey = "ATqSvW7/////AAAAGSbUe3emc0NmiwFnhuicf+c4388daBpHukK2NzjANrVvP6h1rJRTExnNRD8RBZJqsu5tuVVn+AuayqO2UEZbxp0+ZUbFnXPssyKkV4q8YmYB4AkxHwaJCIxCdd1cCWR9F0xuvve5OOzddkh13v/3T1DSh7vrBuFHurMZF8SLQrwQqMf5ubyit0BRHbtX5GLWwm6hCEOX8ZykrK0UbA8+kyGwSqkWbb5IjUMQrlQpItk1emrxo0S2Mj7z+LCNXBNw9wPvTs4TXnpHvcA/7W0vGFxnbXUcUArUBztNHywpD+rVHjFZYuGJwMsWfHAFKH/OfePAstqGnl3GSJjCrEJqVujQo1cqmC7NmyWd2gxPnqHK";

        //What to display on top of visual matches?
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        //Create Vuforia instance with params: -- takes 1-2s
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);

        VuforiaTrackables vuMarks = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable vuMarkTemplate = vuMarks.get(0);

        telemetry.addLine("Status: Initialized and ready!");
        telemetry.update();

        waitForStart();
        vuMarks.activate();

        while (opModeIsActive()) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(vuMarkTemplate);
            telemetry.addLine("VUMARK: " + vuMark);
            telemetry.update();
            idle();
        }
        // Do something useful
    }
}