package org.firstinspires.ftc.teamcode.MainBot.autonomous.Tests.Visual;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.io.File;
import java.io.FileOutputStream;

import static com.vuforia.PIXEL_FORMAT.RGB565;

@Disabled
@Autonomous(name = "aJewel Director", group = "Main Bot")
public class JewelView extends LinearOpMode {

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
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_OBJECT_TARGETS, 8);

        Vuforia.setFrameFormat(RGB565, true);
        vuforia.setFrameQueueCapacity(1);



        /* ************INIT ASSETS:************* */

        telemetry.addLine("Initializing Jewel Assets");
        telemetry.update();

        //Prepares tracking asset group -- takes 1s
        VuforiaTrackables jewels = vuforia.loadTrackablesFromAsset("Pictographs");

        telemetry.addLine("Status: Initialized and ready!");
        telemetry.update();

        VuforiaTrackable jewel = null;

        //BEGIN USER CODE









        //END USER CODE
        waitForStart();
        jewels.activate();
        double waitUntil = time + 1;
        while (time < waitUntil) {
            idle();
        }
        telemetry.addLine("Started");
        telemetry.update();
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
        telemetry.addLine("Taken picture");
        telemetry.update();
        //iterate to find rgb565
        for (int i = 0; i < frame.getNumImages(); i++) {
            //telemetry.addLine("In for loop");
            //elemetry.update();
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                String[] textImage = new String[30];
                //Mat mMat = new Mat();
                Bitmap srcBmp = Bitmap.createBitmap(frame.getImage(i).getWidth(), frame.getImage(i).getHeight(), Bitmap.Config.RGB_565);
                srcBmp.copyPixelsFromBuffer(frame.getImage(i).getPixels());
                telemetry.addLine("Height: " + srcBmp.getHeight());
                telemetry.addLine("Width: " + srcBmp.getWidth());
                telemetry.addLine("Modified Height: " + (srcBmp.getWidth() * (30d/srcBmp.getHeight())));
                telemetry.update();




                Bitmap outBmp = Bitmap.createBitmap(Bitmap.createScaledBitmap(srcBmp, (int) (srcBmp.getWidth() * (30d/srcBmp.getHeight())), 30, false), 0, 0, 30, 30);
                frame.getImage(i).getPixels();

                //Color.blue(outBmp.getPixel(20, 20));



                //Utils.matToBitmap(mMat, outBmp);
                try {
                    File file = new File(Environment.getExternalStorageDirectory().getAbsolutePath(), "Output_" + Math.random() + ".png");
                    FileOutputStream outStream = new FileOutputStream(file);
                    outBmp.compress(Bitmap.CompressFormat.PNG, 100, outStream);
                    outStream.close();
                } catch (Exception e) {
                    telemetry.addLine(e.toString());
                    telemetry.update();
                }
                // telemetric chart used to get color values while debugging
                for (int fx = 0; fx  < 30; fx++) {
                    textImage[29-fx] = "";
                    for (int fy = 0; fy < 30; fy++) {
                        if (Color.blue(outBmp.getPixel(fy, fx)) < 25) {
                            textImage[29-fx] += 'A';
                        } else if (Color.blue(outBmp.getPixel(fy, fx)) < 50) {
                            textImage[29-fx] += 'B';
                        } else if (Color.blue(outBmp.getPixel(fy, fx)) < 90) {
                            textImage[29-fx] += 'C';
                        } else if (Color.blue(outBmp.getPixel(fy, fx)) < 120) {
                            textImage[29-fx] += 'D';
                        } else if (Color.blue(outBmp.getPixel(fy, fx)) < 150) {
                            textImage[29-fx] += 'E';
                        } else if (Color.blue(outBmp.getPixel(fy, fx)) < 170) {
                            textImage[29-fx] += 'F';
                        } else {
                            textImage[29-fx] += 'G';
                        }
                    }
                }
                telemetry.addLine("Blue");
                for (String line: textImage) {
                    telemetry.addLine(line);
                }
                telemetry.update();

                waitUntil = time + 20;
                while (time < waitUntil)
                    idle();
                /*telemetry.addLine("Finished taking picture");
                telemetry.update();*/
            }
        }
    }
}