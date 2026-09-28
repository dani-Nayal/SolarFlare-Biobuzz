package org.firstinspires.ftc.teamcode.vision.testing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.Pollen;
import org.firstinspires.ftc.teamcode.vision.Vision;

import java.util.List;

@TeleOp
public class PollenDetection extends OpMode {
    Vision vision;
    @Override
    public void init() {
        vision = new Vision(hardwareMap, telemetry);
    }

    @Override
    public void start(){
    }

    @Override
    public void loop() {
        List<Pollen> pollen = vision.getPollenWithPythonColor(new Pose(0, 0, 90));

        for (Pollen p : pollen){
            telemetry.addData("x", p.x);
            telemetry.addData("y", p.y);
            telemetry.addData("concentration", p.concentration);
            telemetry.addLine(p.isValid ? "valid" : "invalid");
        }

        LLResult result = vision.limelight.getLatestResult();
        telemetry.addData("result null?", result == null);

        if (result != null) {
            double[] raw = result.getPythonOutput();
            StringBuilder sb = new StringBuilder();
            for (double d : raw) sb.append(d).append(", ");
            telemetry.addData("raw full array", sb.toString());

            List<int[]> decoded = Pollen.decodeBoxes(raw);
            telemetry.addData("decoded box count", decoded.size());
        }


        telemetry.addData("pollen count", pollen.size());
        telemetry.update();

    }
}