package org.firstinspires.ftc.teamcode.vision.testing;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.vision.Pollen;
import org.firstinspires.ftc.teamcode.vision.Vision;

import java.util.List;

@TeleOp
@Utility
public class TestGobildaCameraDetection extends OpMode {
    PoseFactory factory = PoseFactory.degrees();
    Pose testingPose = factory.of(72, 0, 90);
    Vision vision;

    @Override
    public void init() {
        vision = new Vision(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        List<Pollen> pollen = vision.getPollenWithGobildaCamera(testingPose);

    }
}
