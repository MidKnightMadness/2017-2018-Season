package org.firstinspires.ftc.teamcode.MainBot.autonomous.Tests.OlderTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.MainBot.autonomous.VisualController;

@Disabled
@Autonomous(name = "VuforiaDriveTest", group = "Test")
public class VuforiaDriveTest extends OpMode {

    private DriveAssemblyController d = new DriveAssemblyController();
    private VisualController v = new VisualController();

    public VuforiaDriveTest() {
        super();
    }

    @Override
    public void init() {
        d.init(telemetry, hardwareMap);
        v.init(telemetry, hardwareMap);
    }

    @Override
    public void start() {}

    @Override
    public void loop() {
        OpenGLMatrix pos = v.pos();
        if (pos != null) {
            VectorF trans = pos.getTranslation();
            Orientation rot = Orientation.getOrientation(pos, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            d.loop(-(trans.get(1) + 5), -(trans.get(2) + 15), rot.firstAngle);
            telemetry.addData("Trans", trans);
            telemetry.addData("Rot", rot);
        } else {
            telemetry.addLine("NULL");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
    }
}