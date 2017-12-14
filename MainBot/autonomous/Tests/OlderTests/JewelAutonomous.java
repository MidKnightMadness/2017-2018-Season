package org.firstinspires.ftc.teamcode.MainBot.autonomous.Tests.OlderTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MainBot.teleop.CrossCommunicator;

/**
 * Created by Brendan Peercy and Nisha Prasad (Sort of) on 10/10/17.
 */
// THE GRAMMAR IS TERRIBLE
@Disabled
@Autonomous(name = "Jewel Test")
public class JewelAutonomous extends LinearOpMode {
    private DcMotor motor;
    double power = .3;
    int distance = 529;


    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.dcMotor.get(CrossCommunicator.Jewel.MOTOR);
        motor.resetDeviceConfigurationForOpMode();
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        motor.setTargetPosition(motor.getCurrentPosition() - distance);
        motor.setPower(power);
        while (motor.isBusy()) {idle();}

        double waitUntil = time + 2;
        while (time < waitUntil) {
            idle();
        }

        motor.setTargetPosition(motor.getCurrentPosition() + distance);
        motor.setPower(-power);
        while (motor.isBusy()) {idle();}

    }

}
