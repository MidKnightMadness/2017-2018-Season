package org.firstinspires.ftc.teamcode.MainBot.autonomous.Tests.Visual;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CloseableFrame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import java.io.File;
import java.io.FileOutputStream;
import java.util.Arrays;

import static com.vuforia.PIXEL_FORMAT.RGB565;

@Disabled
@Autonomous(name = "aMain Bot Jewel Visual", group = "Main Bot")
public class JewelViewer extends LinearOpMode {

    // whether or not to save the cropped image
    private final boolean SAVE_CROPPED = false;

    // display textual representation of image and wait one second or not
    private final boolean SHOW_TEXT_IMAGE = false;

    //How many times to run through? Suggested: 7
    private final int NUM_OF_TAKES = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        // to keep track of how long each init step takes
        double[] times = new double[7];

        //Log time
        times[0] = time; // = 0s

        /* ************INIT VUFORIA:************* */

        telemetry.addLine("Initializing Vuforia");
        telemetry.update();

        //init the VuforiaLocalizer parameters object
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);

        //Which side of the phone?
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //License Key
        params.vuforiaLicenseKey = "ATqSvW7/////AAAAGSbUe3emc0NmiwFnhuicf+c4388daBpHukK2NzjANrVvP6h1rJRTExnNRD8RBZJqsu5tuVVn+AuayqO2UEZbxp0+ZUbFnXPssyKkV4q8YmYB4AkxHwaJCIxCdd1cCWR9F0xuvve5OOzddkh13v/3T1DSh7vrBuFHurMZF8SLQrwQqMf5ubyit0BRHbtX5GLWwm6hCEOX8ZykrK0UbA8+kyGwSqkWbb5IjUMQrlQpItk1emrxo0S2Mj7z+LCNXBNw9wPvTs4TXnpHvcA/7W0vGFxnbXUcUArUBztNHywpD+rVHjFZYuGJwMsWfHAFKH/OfePAstqGnl3GSJjCrEJqVujQo1cqmC7NmyWd2gxPnqHK";

        //Log time
        times[1] = time - times[0]; // = 0s

        //What to display on top of visual matches?
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        //Create Vuforia instance with params: -- takes 1-2s
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);

        //save an rgb565 image for further processing each frame and only save current frame
        Vuforia.setFrameFormat(RGB565, true);
        vuforia.setFrameQueueCapacity(1);

        //Log time
        times[2] = time - times[1]; // = 1-2s


        /* ************INIT PICTOGRAPH ASSETS:************* */

        telemetry.addLine("Initializing Pictograph Assets");
        telemetry.update();

        //Prepares tracking asset group -- takes 1s
        VuforiaTrackables pictographTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");

        //Log time
        times[3] = time - times[2];

        //The number of times through the main loop
        int runCount = 0;



        /* ************STATUS: READY!************* */

        //print times
        telemetry.addLine("VuforiaParams: " + (int) times[1] + "s Vuforia: " + (int) times[2] + "s Assets:" + (int) times[3] + "s Total: " + (int) time + "s.");
        telemetry.addLine("Status: Initialized and ready!");
        telemetry.update();
        waitForStart();

        double startTime = time;



        /* ************BEGIN!************* */

        //Begin tracking pictographs
        pictographTrackables.activate();

        //While opMode is active, iterate through all trackables
        while (opModeIsActive()) {
            for (VuforiaTrackable item : pictographTrackables) {



                /* ************GET POSE AND TRANSLATION OF PICTOGRAPH IF VISIBLE************* */

                //create OpenGLMatrix containing the translation and rotation of the item relative to the robot if it is visible, else null
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) item.getListener()).getPose();

                //if Vuforia can see this item...
                if (pose != null) {

                    //make a VectorF to hold translation values
                    VectorF translation = pose.getTranslation();

                    /* ************DECIDE WHICH JEWEL IS WHERE************* */

                    // calculate on first 5 runs through to have more resilience to shaking
                    if (runCount < NUM_OF_TAKES) {

                        //Get vuforia's raw Pose data to be converted to OpenCV
                        OpenGLMatrix rawPoseV = ((VuforiaTrackableDefaultListener) item.getListener()).getRawPose();

                        //Get position data in matrix form for Tool.projectPoint
                        Matrix34F rawPose = new Matrix34F();
                        float[] poseData = Arrays.copyOfRange(rawPoseV.transposed().getData(), 0, 12);
                        rawPose.setData(poseData);

                        //Get points of corners of jewels based on pictograph in the 2d image Units: Inch
                        float[][] srcPoints = new float[4][2];
                        srcPoints[0] = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(375f, -100f, 0f)).getData();
                        srcPoints[1] = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(150f, -100f, 0f)).getData();
                        srcPoints[2] = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(150f, -250f, 0f)).getData();
                        srcPoints[3] = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(375f, -250f, 0f)).getData();





                        /*srcPoints[0] = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(14f, -1f, 2.5f)).getData();
                        srcPoints[1] = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(3.5f, -1f, 2.5f)).getData();
                        srcPoints[2] = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(3.5f, -4.25f, 2.5f)).getData();
                        srcPoints[3] = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(14f, -4.25f, 2.5f)).getData();*/








                        /* ************Get Image From Camera************* */

                        //get frame: 5 types: 1 rgb and 4 grayscale --> We want rgb
                        CloseableFrame frame = vuforia.getFrameQueue().take();

                        //iterate to find rgb565
                        for (int i = 0; i < frame.getNumImages(); i++) {
                            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                                Image vuforiaImage = frame.getImage(i);

                                // make bitmap (Android default image format) from Vuforia Image
                                Bitmap srcBmp = Bitmap.createBitmap(vuforiaImage.getWidth(), vuforiaImage.getHeight(), Bitmap.Config.RGB_565);
                                srcBmp.copyPixelsFromBuffer(vuforiaImage.getPixels());

                                float x = Math.min(srcBmp.getWidth(), Math.max(0,
                                        Math.min(Math.min(srcPoints[0][0], srcPoints[1][0]), Math.min(srcPoints[2][0], srcPoints[3][0]))
                                ));
                                float y = Math.min(srcBmp.getHeight(), Math.max(0,
                                        Math.min(Math.min(srcPoints[0][1], srcPoints[1][1]), Math.min(srcPoints[2][1], srcPoints[3][1]))
                                ));
                                float width = Math.min(srcBmp.getWidth() - x, Math.max(0,
                                        Math.max(Math.max(srcPoints[0][0], srcPoints[1][0]), Math.max(srcPoints[2][0], srcPoints[3][0])) - x
                                ));
                                float height = Math.min(srcBmp.getHeight() - y, Math.max(0,
                                        Math.max(Math.max(srcPoints[0][1], srcPoints[1][1]), Math.max(srcPoints[2][1], srcPoints[3][1])) - y
                                ));
                                telemetry.addLine(x + ", " + y + ", " + width + ", " + height);
                                telemetry.update();

                                Bitmap medBmp = Bitmap.createBitmap(srcBmp, (int) x, (int) y, (int) width, (int) height);

                                //If SAVE_CROPPED, save outBmp as png
                                if (SAVE_CROPPED) {

                                    try {

                                        File file = new File("/storage/self/primary/Pictures/images/", "Outpu_" + time + ".png");
                                        FileOutputStream outStream = new FileOutputStream(file);
                                        medBmp.compress(Bitmap.CompressFormat.PNG, 100, outStream);
                                        outStream.close();
                                    } catch (Exception e) {
                                        telemetry.addLine(e.toString());
                                        telemetry.update();
                                    }
                                }


                                telemetry.addLine("MedBMP!");
                                telemetry.update();
                                Bitmap outBmp = Bitmap.createScaledBitmap(medBmp, 2, 4, false);


                                //Check color
                                int blueCount = 0, redCount = 0;
                                double[][] getColor = new double[4][2];
                                for (i = 0; i < 4; i++) {
                                    for (int j = 0; j < 1; j++) {
                                        int blueVal, redVal, difference;
                                        blueVal = Color.blue(outBmp.getPixel(j, i));
                                        redVal = Color.red(outBmp.getPixel(j, i));
                                        difference = redVal - blueVal;
                                        if (Math.abs(difference) < 40) {
                                            break;
                                        }
                                        else if (difference < 0) {
                                            blueCount++;
                                        }
                                        else {
                                            redCount++;
                                        }
                                        /*telemetry.addLine("Ratio: " + i + " " + j + " ");
                                        telemetry.addLine(redVal + ":" + blueVal);*/
                                    }
                                }
                                String left;
                                String right;
                                telemetry.addLine("Column 0: " + redCount + " " + blueCount);
                                if (redCount > blueCount) {
                                    telemetry.addLine("Red left");
                                    left = "Red";
                                }
                                else {
                                    telemetry.addLine("Blue left");
                                    left = "Blue";
                                }
                                redCount = 0;
                                blueCount = 0;
                                for (i = 0; i < 4; i++) {
                                    for (int j = 1; j < 2; j++) {
                                        int blueVal, redVal, difference;
                                        blueVal = Color.blue(outBmp.getPixel(j, i));
                                        redVal = Color.red(outBmp.getPixel(j, i));
                                        difference = redVal - blueVal;
                                        if (Math.abs(difference) < 40) {
                                            break;
                                        }
                                        else if (difference < 0) {
                                            blueCount++;

                                        }
                                        else {
                                            redCount++;
                                        }
                                        /*telemetry.addLine("Ratio: " + i + " " + j + " ");
                                        telemetry.addLine(redVal + ":" + blueVal);*/
                                    }
                                }
                                telemetry.addLine("Column 1: " + redCount + " " + blueCount);
                                if (redCount > blueCount) {
                                    telemetry.addLine("Red right");
                                    right = "Red";
                                }
                                else {
                                    telemetry.addLine("Blue right");
                                    right = "Blue";
                                }
                                if (left != right) {
                                    telemetry.addLine("Yay");
                                } else {
                                    continue;
                                }
                                telemetry.addLine("Pictograph: " + RelicRecoveryVuMark.from(pictographTrackables.get(0)));
                                telemetry.update();





                                //If SAVE_CROPPED, save outBmp as png
                                if (SAVE_CROPPED) {

                                    try {

                                        File file = new File("/storage/self/primary/Pictures/images/", "Outpu_" + time + ".png");
                                        FileOutputStream outStream = new FileOutputStream(file);
                                        outBmp.compress(Bitmap.CompressFormat.PNG, 100, outStream);
                                        outStream.close();
                                    } catch (Exception e) {
                                        telemetry.addLine(e.toString());
                                        telemetry.update();
                                    }
                                }
                            }
                        }
                    }
                    runCount++;
                }
            }

            idle();
        }
    }
}