package org.firstinspires.ftc.teamcode.vision.testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestLLResult extends OpMode {
    Limelight3A limelight;
    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(11);
        limelight.pipelineSwitch(1);
    }

    @Override
    public void start(){
        limelight.start();
    }

    @Override
    public void loop(){
        LLResult result = limelight.getLatestResult();

        telemetry.addData("is connected?", limelight.isConnected());
        telemetry.addData("is null?", result == null);
        telemetry.addData("is running?", limelight.isRunning());

        if (result != null){
            double[] boundingBoxes = result.getPythonOutput();

            telemetry.addData("bounding boxes", boundingBoxes);
            telemetry.addData("num items", boundingBoxes.length);
        }
        else {
            telemetry.addLine("not working");
        }
        telemetry.update();
    }

}
