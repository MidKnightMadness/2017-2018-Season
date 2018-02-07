package org.firstinspires.ftc.teamcode.MainBot.teleop.SimpleDriveAssembly;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

/**
 * Created by nisha.prasad on 10/17/17.
 */

public class SimpleDriveAssemblyController {
    private DcMotor fl;
    private DcMotor br;
    private DcMotor bl;
    private DcMotor fr;


    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        //fl
        fl = hardwareMap.dcMotor.get(CrossCommunicator.Drive.FRONT_LEFT);
        fl.resetDeviceConfigurationForOpMode();
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //br
        br = hardwareMap.dcMotor.get(CrossCommunicator.Drive.BACK_RIGHT);
        br.resetDeviceConfigurationForOpMode();
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bl = hardwareMap.dcMotor.get(CrossCommunicator.Drive.BACK_LEFT);
        bl.resetDeviceConfigurationForOpMode();
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fr = hardwareMap.dcMotor.get(CrossCommunicator.Drive.FRONT_RIGHT);
        fr.resetDeviceConfigurationForOpMode();
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void start() {
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double r = gamepad1.right_stick_x;

        fl.setPower(clip(-x-y+r));
        br.setPower(clip(x+y+r));
        bl.setPower(clip(x-y+r));
        fr.setPower(clip(-x+y+r));
    }

    public void  stop() {
    }

    private double clip (double value) {
        if (value > 1) {
            value = 1;
        }
        else if(value < -1) {
            value = -1;
        }
        return value;
    }
}
