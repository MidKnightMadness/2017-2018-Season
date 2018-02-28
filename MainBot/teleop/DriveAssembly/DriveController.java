package org.firstinspires.ftc.teamcode.OldMainBot.teleop.DriveAssembly;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OldMainBot.teleop.CrossCommunicator;

public class DriveController {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.dcMotor.get(CrossCommunicator.Drive.MOTOR_FRONT_LEFT);
        frontLeftMotor.resetDeviceConfigurationForOpMode();
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Front Left Power", new Func<Double>() {
            @Override
            public Double value() {
                return frontLeftMotor.getPower();
            }
        });
        telemetry.addData("Front Left Position", new Func<Integer>() {
            @Override
            public Integer value() {
                return frontLeftMotor.getCurrentPosition();
            }
        });
        telemetry.addData("Front Left Target", new Func<Integer>() {
            @Override
            public Integer value() {
                return frontLeftMotor.getTargetPosition();
            }
        });

        frontRightMotor = hardwareMap.dcMotor.get(CrossCommunicator.Drive.MOTOR_FRONT_RIGHT);
        frontRightMotor.resetDeviceConfigurationForOpMode();
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Front Right Power", new Func<Double>() {
            @Override
            public Double value() {
                return frontRightMotor.getPower();
            }
        });
        telemetry.addData("Front Right Position", new Func<Integer>() {
            @Override
            public Integer value() {
                return frontRightMotor.getCurrentPosition();
            }
        });
        telemetry.addData("Front Right Target", new Func<Integer>() {
            @Override
            public Integer value() {
                return frontRightMotor.getTargetPosition();
            }
        });

        backLeftMotor = hardwareMap.dcMotor.get(CrossCommunicator.Drive.MOTOR_BACK_LEFT);
        backLeftMotor.resetDeviceConfigurationForOpMode();
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Back Left Power", new Func<Double>() {
            @Override
            public Double value() {
                return backLeftMotor.getPower();
            }
        });
        telemetry.addData("Back Left Position", new Func<Integer>() {
            @Override
            public Integer value() {
                return backLeftMotor.getCurrentPosition();
            }
        });
        telemetry.addData("Back Left Target", new Func<Integer>() {
            @Override
            public Integer value() {
                return backLeftMotor.getTargetPosition();
            }
        });

        backRightMotor = hardwareMap.dcMotor.get(CrossCommunicator.Drive.MOTOR_BACK_RIGHT);
        backRightMotor.resetDeviceConfigurationForOpMode();
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Back Right Power", new Func<Double>() {
            @Override
            public Double value() {
                return backRightMotor.getPower();
            }
        });
        telemetry.addData("Back Right Position", new Func<Integer>() {
            @Override
            public Integer value() {
                return backRightMotor.getCurrentPosition();
            }
        });
        telemetry.addData("Back Right Target", new Func<Integer>() {
            @Override
            public Integer value() {
                return backRightMotor.getTargetPosition();
            }
        });
    }

    public void start() {
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        float joyLX = scaleInput(-gamepad1.left_stick_x);
        float joyLY = scaleInput(-gamepad1.left_stick_y);
        float joyRX = scaleInput(-gamepad1.right_stick_x);
        float joyRY = scaleInput(-gamepad1.right_stick_y);
        float modX = -(joyLX + joyRX) / 2;

        frontLeftMotor.setPower(CrossCommunicator.mainBotThrottleFactor * Range.clip(modX + joyLY, -1F, 1F));
        frontRightMotor.setPower(CrossCommunicator.mainBotThrottleFactor * Range.clip(joyRY - modX, -1F, 1F));
        backLeftMotor.setPower(CrossCommunicator.mainBotThrottleFactor * Range.clip(joyLY - modX, -1F, 1F));
        backRightMotor.setPower(CrossCommunicator.mainBotThrottleFactor * Range.clip(modX + joyRY, -1F, 1F));
    }

    public void  stop() {
    }

    private float scaleInput(float input) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24, 0.30, 0.36, 0.43,
                0.50, 0.60, 0.72, 0.85, 1.0, 1.0};
        int index = (int) (input * 16);
        if (index < 0)
            index = -index;
        else if (index > 16)
            index = 16;
        float scaled;
        if (input < 0)
            scaled = (float) -scaleArray[index];
        else
            scaled = (float) scaleArray[index];
        return scaled / 2;
    }
}
