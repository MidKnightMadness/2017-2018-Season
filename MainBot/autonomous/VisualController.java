package org.firstinspires.ftc.teamcode.MainBot.autonomous;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.io.File;
import java.io.FileOutputStream;
import java.util.Arrays;
import java.util.Locale;

/**
 * Created by gregory.ling on 10/15/17.
 */

public class VisualController {
    // whether or not to save the cropped image
    private boolean SAVE_CROPPED = true;

    public enum JewelColor {
        RED,
        BLUE
    }

    public JewelColor rightJewel;
    public RelicRecoveryVuMark pictograph;

    private Telemetry telemetry;
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables pictographTrackables;

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;

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
        vuforia = ClassFactory.createVuforiaLocalizer(params);

        //save an rgb565 image for further processing each frame and only save current frame
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);

        /* ************INIT PICTOGRAPH ASSETS:************* */

        telemetry.addLine("Initializing Pictograph Assets");
        telemetry.update();

        //Prepares tracking asset group -- takes 1s
        pictographTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");

        //Enable flash
        //CameraDevice.getInstance().setFlashTorchMode(true);
        pictographTrackables.activate();

    }

    public void look() throws InterruptedException {

        /* ************BEGIN!************* */
        //Begin tracking pictographs
        //boolean pictographFound = false;
        //While opMode is active, iterate through all trackables
        //while (!pictographFound) {
            for (VuforiaTrackable item : pictographTrackables) {
                /* ************DECIDE WHICH JEWEL IS WHERE************* */
                // calculate on first 5 runs through to have more resilience to shaking
                OpenGLMatrix rawPoseV = ((VuforiaTrackableDefaultListener) item.getListener()).getRawPose();
                if (rawPoseV != null) {
                    //Get position data in matrix form for Tool.projectPoint
                    Matrix34F rawPose = new Matrix34F();
                    float[] poseData = Arrays.copyOfRange(rawPoseV.transposed().getData(), 0, 12);
                    rawPose.setData(poseData);
                    //Get points of corners of jewels based on pictograph in the 2d image Units: Inch?
                    float[][] srcPoints = new float[4][2];
                    srcPoints[0] = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(375f, -100f, 0f)).getData();
                    srcPoints[1] = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(100f, -100f, 0f)).getData();
                    srcPoints[2] = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(100f, -250f, 0f)).getData();
                    srcPoints[3] = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(375f, -250f, 0f)).getData();
                    /* ************Get Image From Camera************* */
                    //get frame: 5 types: 1 rgb and 4 grayscale --> We want rgb
                    VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
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
                            Bitmap outBmp;
                            if (SAVE_CROPPED) {
                                try {
                                    File file = new File("/storage/self/primary/Pictures/images/", "Output_S" + Math.random() + ".png");
                                    FileOutputStream outStream = new FileOutputStream(file);
                                    srcBmp.compress(Bitmap.CompressFormat.PNG, 100, outStream);
                                    outStream.close();
                                } catch (Exception e) {
                                    //telemetry.addLine(e.toString());
                                    //telemetry.update();
                                }
                                Bitmap medBmp = Bitmap.createBitmap(srcBmp, (int) x, (int) y, (int) width, (int) height);
                                try {
                                    File file = new File("/storage/self/primary/Pictures/images/", "Output_M" + Math.random() + ".png");
                                    FileOutputStream outStream = new FileOutputStream(file);
                                    medBmp.compress(Bitmap.CompressFormat.PNG, 100, outStream);
                                    outStream.close();
                                } catch (Exception e) {
                                    //telemetry.addLine(e.toString());
                                    //telemetry.update();
                                }
                                //telemetry.addLine("MedBMP!");
                                //telemetry.update();
                                outBmp = Bitmap.createScaledBitmap(medBmp, 2, 4, false);
                                try {
                                    File file = new File("/storage/self/primary/Pictures/images/", "Output_O" + Math.random() + ".png");
                                    FileOutputStream outStream = new FileOutputStream(file);
                                    outBmp.compress(Bitmap.CompressFormat.PNG, 100, outStream);
                                    outStream.close();
                                } catch (Exception e) {
                                    //telemetry.addLine(e.toString());
                                    //telemetry.update();
                                }
                            } else {
                                outBmp = Bitmap.createScaledBitmap(Bitmap.createBitmap(srcBmp, (int) x, (int) y, (int) width, (int) height), 2, 4, false);
                            }


                            /*//Check color
                            int blueCount = 0, redCount = 0, difference;
                            for (i = 0; i < 4; i++) {
                                for (int j = 0; j < 1; j++) {
                                    difference = Color.red(outBmp.getPixel(j, i)) - Color.blue(outBmp.getPixel(j, i));
                                    if (Math.abs(difference) < 40) {
                                        break;
                                    } else if (difference < 0) {
                                        blueCount++;
                                    } else {
                                        redCount++;
                                    }
                                }
                            }*/
                            // BEGIN EDIT:
                            int redCount = 0;
                            int blueCount = 0;
                            for (int j = 0; j < 4; j++) {
                                redCount += Color.red(outBmp.getPixel(0, j));
                                blueCount += Color.blue(outBmp.getPixel(0, j));
                            }

                            //END EDIT:
                            JewelColor left, right;
                            //telemetry.addLine("Column 0: " + redCount + " " + blueCount);
                            if (redCount > blueCount) {
                                //telemetry.addLine("Red left");
                                left = JewelColor.RED;
                            } else {
                                //telemetry.addLine("Blue left");
                                left = JewelColor.BLUE;
                            }
                            /*
                            redCount = 0;
                            blueCount = 0;
                            for (i = 0; i < 4; i++) {
                                for (int j = 1; j < 2; j++) {
                                    difference = Color.red(outBmp.getPixel(j, i)) - Color.blue(outBmp.getPixel(j, i));
                                    if (Math.abs(difference) < 40) {
                                        break;
                                    } else if (difference < 0) {
                                        blueCount++;
                                    } else {
                                        redCount++;
                                    }
                                }
                            }*/

                            // BEGIN EDIT:
                            redCount = 0;
                            blueCount = 0;
                            for (int j = 0; j < 4; j++) {
                                redCount += Color.red(outBmp.getPixel(1, j));
                                blueCount += Color.blue(outBmp.getPixel(1, j));
                            }

                            //END EDIT:

                            //telemetry.addLine("Column 1: " + redCount + " " + blueCount);
                            if (redCount > blueCount) {
                                //telemetry.addLine("Red right");
                                right = JewelColor.RED;
                            } else {
                                //telemetry.addLine("Blue right");
                                right = JewelColor.BLUE;
                            }

                            if (left != right) {
                                //telemetry.addLine("Yay");
                                rightJewel = left;
                            } else {
                                continue;
                            }
                            pictograph = RelicRecoveryVuMark.from(pictographTrackables.get(0));
                            //pictographFound = true;

                            //telemetry.addLine("Pictograph: " + RelicRecoveryVuMark.from(pictographTrackables.get(0)));
                            //telemetry.update();
                            break;
                        }
                    }
                }
            }
        //}
        //CameraDevice.getInstance().setFlashTorchMode(false);
    }








    public void blindLook() throws InterruptedException {

        /* ************BEGIN!************* */
        //Begin tracking pictographs
        //boolean pictographFound = false;
        //While opMode is active, iterate through all trackables
        //while (!pictographFound) {
        telemetry.addLine("a");
        telemetry.update();
                /* ************DECIDE WHICH JEWEL IS WHERE************* */
            // calculate on first 5 runs through to have more resilience to shaking
                //Get position data in matrix form for Tool.projectPoint
                //Get points of corners of jewels based on pictograph in the 2d image Units: Inch?
                    /* ************Get Image From Camera************* */
                //get frame: 5 types: 1 rgb and 4 grayscale --> We want rgb
                VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
                telemetry.addLine("pic taken");
                telemetry.update();
                //iterate to find rgb565
                for (int i = 0; i < frame.getNumImages(); i++) {
                    if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                        Image vuforiaImage = frame.getImage(i);
                        // make bitmap (Android default image format) from Vuforia Image
                        Bitmap srcBmp = Bitmap.createBitmap(vuforiaImage.getWidth(), vuforiaImage.getHeight(), Bitmap.Config.RGB_565);
                        srcBmp.copyPixelsFromBuffer(vuforiaImage.getPixels());
                        telemetry.update();
                        Bitmap outBmp;
                        //if (SAVE_CROPPED) {
                            try {
                                File file = new File("/storage/self/primary/Pictures/images/", "Output_S" + Math.random() + ".png");
                                FileOutputStream outStream = new FileOutputStream(file);
                                srcBmp.compress(Bitmap.CompressFormat.PNG, 100, outStream);
                                outStream.close();
                            } catch (Exception e) {
                                //telemetry.addLine(e.toString());
                                //telemetry.update();
                            }
                            Bitmap medBmp = Bitmap.createBitmap(srcBmp, 50, 0, 650, 190);
                            try {
                                File file = new File("/storage/self/primary/Pictures/images/", "Output_M" + Math.random() + ".png");
                                FileOutputStream outStream = new FileOutputStream(file);
                                medBmp.compress(Bitmap.CompressFormat.PNG, 100, outStream);
                                outStream.close();
                                telemetry.addLine("we tried");
                                telemetry.update();
                            } catch (Exception e) {
                                telemetry.addLine(e.toString());
                                telemetry.update();
                            }
                            telemetry.addLine("MedBMP!");
                            telemetry.update();
                            outBmp = Bitmap.createScaledBitmap(medBmp, 4, 4, false);
                            try {
                                File file = new File("/storage/self/primary/Pictures/images/", "Output_O" + Math.random() + ".png");
                                FileOutputStream outStream = new FileOutputStream(file);
                                outBmp.compress(Bitmap.CompressFormat.PNG, 100, outStream);
                                outStream.close();
                            } catch (Exception e) {
                                telemetry.addLine(e.toString());
                                telemetry.update();
                            }
                        //} else {
                        //    outBmp = Bitmap.createScaledBitmap(Bitmap.createBitmap(srcBmp, 50, 0, 580, 190), 2, 4, false);
                        //}


                            /*//Check color
                            int blueCount = 0, redCount = 0, difference;
                            for (i = 0; i < 4; i++) {
                                for (int j = 0; j < 1; j++) {
                                    difference = Color.red(outBmp.getPixel(j, i)) - Color.blue(outBmp.getPixel(j, i));
                                    if (Math.abs(difference) < 40) {
                                        break;
                                    } else if (difference < 0) {
                                        blueCount++;
                                    } else {
                                        redCount++;
                                    }
                                }
                            }*/
                        // BEGIN EDIT:

                        int maxRed = 0;
                        int maxRedColumn = -1;
                        int maxBlue = 0;
                        int maxBlueColumn = -1;
                        int columns[][] = new int[4][2];
                        for (int k = 0; k < 4; k++) {
                            for (int j = 0; j < 4; j++) {
                                columns[k][0] += Color.red(outBmp.getPixel(k, j));
                                columns[k][1] += Color.blue(outBmp.getPixel(k, j));
                            }

                            boolean isMaxRed = false;
                            if (columns[k][0] > maxRed && columns[k][0] - columns[k][1] > 50) {
                                maxRed = columns[k][0];
                                maxRedColumn = k;
                            }
                            if (columns[k][1] > maxBlue && columns[k][1] - columns[k][0] > 50) {
                                maxBlue = columns[k][1];
                                maxBlueColumn = k;
                            }
                            telemetry.addLine(columns[k][0] + ", " + columns[k][1]);
                        }
                        if (maxRed > 50 && maxBlue > 50 && maxRedColumn != maxBlueColumn) {
                            if (maxRedColumn > maxBlueColumn) {
                                rightJewel = JewelColor.BLUE;
                            } else {
                                rightJewel = JewelColor.RED;
                            }
                        } else {
                            rightJewel = null;
                            telemetry.addLine(String.format(Locale.ENGLISH, "%d, %d", maxRed, maxBlue));
                        }
                        telemetry.addLine(rightJewel == JewelColor.BLUE ? "BLUE" : rightJewel == JewelColor.RED ? "RED" : "UNKNOWN");
                        telemetry.update();
                        break;
                    }


        }
        //}
        //CameraDevice.getInstance().setFlashTorchMode(false);
    }

    public OpenGLMatrix pos() {
        for (VuforiaTrackable item : pictographTrackables) {
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) item.getListener()).getPose();
            if (pose != null) {
                return pose;
            }
        }
        return null;
    }

    public void saveTeamColor(int team) throws InterruptedException {
        try {
            File file = new File("/storage/self/primary/", "LastTeamColor.txt");
            FileOutputStream outStream = new FileOutputStream(file);
            outStream.write(team);
            outStream.flush();
            outStream.close();
        } catch (Exception e) {
            telemetry.addLine(e.toString());
            telemetry.update();
        }
    }
}
