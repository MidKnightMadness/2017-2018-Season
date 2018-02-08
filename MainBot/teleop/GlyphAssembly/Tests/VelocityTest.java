package org.firstinspires.ftc.teamcode.MainBot.teleop.GlyphAssembly.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStream;
import java.io.OutputStreamWriter;

@TeleOp(name = "Velocity Test", group = "Tests")
public class VelocityTest extends OpMode {

    DcMotor motor;
    ElapsedTime elapsedTime = new ElapsedTime();
    FileOutputStream fileOutputStream;
    OutputStreamWriter outputStreamWriter;
    int lastPosition = 0;
    double lastTime = 0;
    double velocity = 0;
    double thisTime = 0;
    double thisEnc = 0;
    double velocityArray[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int i = 0;
    int target = 1000;


    public VelocityTest() {
        super();
    }

    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void start() {
        try {
            int j = 0;
            while (new File("/storage/self/primary/Data/", j + "Data.txt").exists()) {
                j++;
            }
            File file = new File("/storage/self/primary/Data/", j + "Data.txt");
            fileOutputStream = new FileOutputStream(file);
            outputStreamWriter = new OutputStreamWriter(fileOutputStream);
        } catch (Exception e) {
            telemetry.addData("Error", e);
            telemetry.update();
        }
        elapsedTime.reset();
        //motor.setPower(1);
    }

    @Override
    public void loop() {
        i = i + 1 < 20 ? i + 1 : 0;
        thisEnc = lastPosition - motor.getCurrentPosition();
        lastPosition = motor.getCurrentPosition();
        thisTime = lastTime - elapsedTime.seconds();
        lastTime = elapsedTime.seconds();


        velocityArray[i] = thisEnc / thisTime;
        velocity = 0;
        for (int j = 0; j < 20; j++) {
            velocity += velocityArray[j];
        }
        velocity /= 20;
        //velocity = 0.95 * velocity + 0.05 * (thisEnc / thisTime);
        double power = 0.0005 * (target - motor.getCurrentPosition()) - 0.0001 * velocity;
        telemetry.addData("Velocity (enc/sec)", velocity);
        telemetry.addData("Time This Run", thisTime);
        telemetry.addData("Enc This Run", thisEnc);
        telemetry.addData("Speed", Math.min(Math.max(power
                , -0.5), 0.5));
        telemetry.addData("Unrestricted Speed", power);
        telemetry.update();
        try {
            //outputStreamWriter.append(velocity + "\n");
        } catch (Exception e) {

        }
        if (gamepad1.a) {
            target = 1000;
        } else {
            target = 0;
        }

        motor.setPower(Math.min(Math.max(
                power
                , -0.5), 0.5));
    }

    @Override
    public void stop() {
        try {
            outputStreamWriter.flush();
            outputStreamWriter.close();
            fileOutputStream.flush();
            fileOutputStream.close();
        } catch (Exception e) {

        }

    }
}