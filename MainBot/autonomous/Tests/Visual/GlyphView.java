package org.firstinspires.ftc.teamcode.MainBot.autonomous.Tests.Visual;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Disabled
@Autonomous(name = "aGlyph Director", group = "Vuforia")
public class GlyphView extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {



        /* ************INIT VUFORIA:************* */

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
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_OBJECT_TARGETS, 24);



        /* ************INIT ASSETS:************* */

        telemetry.addLine("Initializing Pictograph Assets");
        telemetry.update();

        //Prepares tracking asset group -- takes 1s
        VuforiaTrackables glyphs = vuforia.loadTrackablesFromAsset("Glyphs");

        telemetry.addLine("Status: Initialized and ready!");
        telemetry.update();

        VuforiaTrackable glyph = null;

        //BEGIN USER CODE









        //END USER CODE
        waitForStart();


        glyphs.activate();

        while(opModeIsActive()) {
            if (glyph != null && ((VuforiaTrackableDefaultListener) glyph.getListener()).getPose() != null) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) glyph.getListener()).getPose();

                VectorF translation = pose.getTranslation();

                telemetry.addLine("X: " + translation.get(0) + "Y: " + translation.get(1) + "Z: " + translation.get(2));
                /*     y
                    -------*  <- glyph
                    |     /
                    |    /
                  x |   / d
                    |  /
                    |-/
                    |/
                    @  <- phone


                    Find d and measure of angle D

                    Drive toward the glyph represented in translation until d < 3 using Jerry
                    all units are inches and x and y is relative to the center of the glyph

                    only change code in user code zones


                    GO!
                */

                //BEGIN USER CODE












                //END USER CODE

            } else {
                for (VuforiaTrackable tempGlyph : glyphs) {
                    if (((VuforiaTrackableDefaultListener) tempGlyph.getListener()).getPose() != null) {
                        glyph = tempGlyph;
                    }
                }
            }
        }
    }
}