package org.firstinspires.ftc.teamcode.vision;
import static java.lang.Double.NaN;

import com.bylazar.field.FieldManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.List;

public class Vision {
    public final double INTAKING_WIDTH_INCHES = 10;
    public CAMERA_ORIENTATION cameraOrientation = CAMERA_ORIENTATION.NORMAL;
    public final int NN_PIPELINE_INDEX = 0;
    public final int PYTHON_COLOR_PIPELINE_INDEX = 1;
    public final int APRIL_TAGS_PIPELINE_INDEX = 2;
    public final double HORIZONTAL_FOV = 54.4;
    public Pose3D cameraPoseOnRobot = new Pose3D(new Position(DistanceUnit.INCH, 3, -5.5, 5.7, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
    public Limelight3A limelight;
    public Telemetry telemetry;
    List<Integer> localizationAprilTags = new ArrayList<>();

    public enum CAMERA_ORIENTATION {
        NORMAL,
        UPSIDE_DOWN,
        CLOCKWISE_90,
        COUNTER_CLOCKWISE_90
    }

    public Vision(HardwareMap hardwareMap, Telemetry telemetry){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.telemetry = telemetry;
        limelight.setPollRateHz(11);
        localizationAprilTags.add(20);
        localizationAprilTags.add(24);
    }

    public Vision(HardwareMap hardwareMap, Telemetry telemetry, Pose3D cameraPoseOnRobot, CAMERA_ORIENTATION cameraOrientation){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.telemetry = telemetry;
        limelight.setPollRateHz(11);
        this.cameraPoseOnRobot = cameraPoseOnRobot;
        this.cameraOrientation = cameraOrientation;
        localizationAprilTags.add(20);
        localizationAprilTags.add(24);
    }

    public List<Pollen> getPollenWithPythonColor(Pose botPose){
        if (!limelight.isRunning()) limelight.start();
        if (limelight.getStatus().getPipelineIndex() != PYTHON_COLOR_PIPELINE_INDEX) limelight.pipelineSwitch(PYTHON_COLOR_PIPELINE_INDEX);

        List<Pollen> out = new ArrayList<>();

        LLResult result = limelight.getLatestResult();

        if (result != null){
            double[] pythonOutput = result.getPythonOutput();

            List<int[]> decodedOutput = Pollen.decodeBoxes(pythonOutput);

            for (int[] outPut : decodedOutput) {
                Pollen pollen = new Pollen(outPut[0], outPut[1], outPut[2], outPut[3], botPose, cameraPoseOnRobot);

                out.add(pollen);
            }
        }
        return out;
    }

    public List<Pollen> getPollenWithNN(Pose botPose){
        if (!limelight.isRunning()) limelight.start();
        if (limelight.getStatus().getPipelineIndex() != NN_PIPELINE_INDEX) limelight.pipelineSwitch(NN_PIPELINE_INDEX);

        List<Pollen> out = new ArrayList<>();

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()){
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

            for (LLResultTypes.DetectorResult detection : detections){
                Pollen pollen = new Pollen(detection, cameraOrientation, botPose, cameraPoseOnRobot);

                if (pollen.isValid) out.add(pollen);
            }
        }
        return out;
    }

    public Pose getBotPoseMT1(Pose odometryPose){
        if (!limelight.isRunning()) limelight.start();
        if (limelight.getStatus().getPipelineIndex() != APRIL_TAGS_PIPELINE_INDEX) limelight.pipelineSwitch(APRIL_TAGS_PIPELINE_INDEX);

        double odometryHeading = normalizeHeading360Degrees(Math.toDegrees(odometryPose.getHeading()));

        Pose out = null;

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()){
            List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();

            for (LLResultTypes.FiducialResult aprilTag : aprilTags){
                if (localizationAprilTags.contains(aprilTag.getFiducialId())){
                    Pose aprilTagPose = limelightToPedroPose(result.getBotpose());

                    double aprilTagHeading = Math.toDegrees(aprilTagPose.getHeading());

                    double diff = (aprilTagHeading - odometryHeading + 540) % 360 - 180;
                    double headingDiff = Math.abs(diff);

                    double dist = Math.hypot(aprilTagPose.getX() - odometryPose.getX(), aprilTagPose.getY() - odometryPose.getY());

                    if ((headingDiff < 15) && (dist < 10)){
                        out = aprilTagPose;
                    }
                }
            }
        }
        else return null;

        return out;
    }

    public Pose getBotPoseMT2(Pose odometryPose){
        if (!limelight.isRunning()) limelight.start();
        if (limelight.getStatus().getPipelineIndex() != APRIL_TAGS_PIPELINE_INDEX) limelight.pipelineSwitch(APRIL_TAGS_PIPELINE_INDEX);

        double odometryHeading = normalizeHeading360Degrees(Math.toDegrees(odometryPose.getHeading()));

        limelight.updateRobotOrientation(standardToLimelightYaw(odometryHeading));

        Pose out = null;

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()){
            List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();

            for (LLResultTypes.FiducialResult aprilTag : aprilTags){
                if (localizationAprilTags.contains(aprilTag.getFiducialId())){
                    Pose aprilTagPose = limelightToPedroPose(result.getBotpose());

                    double aprilTagHeading = Math.toDegrees(aprilTagPose.getHeading());

                    double diff = (aprilTagHeading - odometryHeading + 540) % 360 - 180;
                    double headingDiff = Math.abs(diff);

                    double dist = Math.hypot(aprilTagPose.getX() - odometryPose.getX(), aprilTagPose.getY() - odometryPose.getY());

                    if ((headingDiff < 15) && (dist < 10)){
                        out = aprilTagPose;
                    }
                }
            }
        }
        else return null;

        return out;
    }
    public Pose getBotPoseMT2WithMT1(Pose odometryPose) {
        if (!limelight.isRunning()) limelight.start();
        if (limelight.getStatus().getPipelineIndex() != APRIL_TAGS_PIPELINE_INDEX) limelight.pipelineSwitch(APRIL_TAGS_PIPELINE_INDEX);

        double odometryHeading = normalizeHeading360Degrees(Math.toDegrees(odometryPose.getHeading()));

        Pose botPoseMT1 = getBotPoseMT1(odometryPose);

        if (botPoseMT1 != null) {
            double llYaw = standardToLimelightYaw(Math.toDegrees(botPoseMT1.getHeading()));

            limelight.updateRobotOrientation(llYaw);

            Pose out = null;

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();

                for (LLResultTypes.FiducialResult aprilTag : aprilTags){
                    if (localizationAprilTags.contains(aprilTag.getFiducialId())){
                        Pose aprilTagPose = limelightToPedroPose(result.getBotpose());

                        double aprilTagHeading = Math.toDegrees(aprilTagPose.getHeading());

                        double diff = (aprilTagHeading - odometryHeading + 540) % 360 - 180;
                        double headingDiff = Math.abs(diff);

                        double dist = Math.hypot(aprilTagPose.getX() - odometryPose.getX(), aprilTagPose.getY() - odometryPose.getY());

                        if ((headingDiff < 15) && (dist < 10)){
                            out = aprilTagPose;
                        }
                    }
                }
            }
            return out;
        }
        return null;
    }

    public Pose limelightToPedroPose(Pose3D llPose){
        double llX = llPose.getPosition().toUnit(DistanceUnit.INCH).x;
        double llY = llPose.getPosition().toUnit(DistanceUnit.INCH).y;
        double llYaw = llPose.getOrientation().getYaw(AngleUnit.DEGREES);

        double xTransformed = llY + 72;
        double yTransformed = 72 - llX;
        double yawTransformed = limelightToStandardYaw(llYaw);

        yawTransformed = Math.toRadians(yawTransformed);

        return new Pose(xTransformed, yTransformed, yawTransformed);
    }

    public double standardToLimelightYaw(double standardYawDegrees){
        double yawTransformed = standardYawDegrees + 90;

        if (yawTransformed >= 180.0) {
            yawTransformed -= 360.0;
        }
        return yawTransformed;
    }

    public double limelightToStandardYaw(double llYawDegrees){
        double yawTransformed = (llYawDegrees + 270) % 360;

        if (yawTransformed < 0) {
            yawTransformed += 360;
        }
        return yawTransformed;
    }
    public void stopLimelight(){
        limelight.stop();
    }

    public Double intakingAngle(List<Pollen> detections, Pose botPose, int stepDeg) {
        if (detections.isEmpty()) return null;

        double bestAngle = NaN;

        double bestScore = Double.NEGATIVE_INFINITY;

        final double MAX_RELEVANT_DISTANCE = 70;

        double w1 = 10;
        double w2 = 4;
        double w3 = 1;

        double horizontalLeftBound = Math.toDegrees(botPose.getHeading()) + (HORIZONTAL_FOV / 2);
        double horizontalRightBound = Math.toDegrees(botPose.getHeading()) - (HORIZONTAL_FOV / 2);

        for (double angleDeg = horizontalLeftBound; angleDeg >= horizontalRightBound; angleDeg -= stepDeg) {

            double theta = Math.toRadians(angleDeg);
            double cos = Math.cos(theta);
            double sin = Math.sin(theta);

            int count = 0;
            double totalPerpendicularDist = 0;
            double totalDist = 0;

            for (Pollen detection : detections) {
                double rx = detection.x - botPose.getX();
                double ry = detection.y - botPose.getY();

                double perpendicularDist = Math.abs(rx * sin - ry * cos);
                double distance = Math.hypot(rx, ry);

                if (perpendicularDist < INTAKING_WIDTH_INCHES / 2) {
                    count++;
                    totalPerpendicularDist += perpendicularDist;
                    totalDist += distance;
                }
            }

            if (count > 0) {
                double avgPerpendicularDist = totalPerpendicularDist / count;
                double avgDist = totalDist / count;

                double normCount = count / (double) detections.size();
                double normPerpendicular = avgPerpendicularDist / (INTAKING_WIDTH_INCHES / 2);
                double normDist = avgDist / MAX_RELEVANT_DISTANCE;

                double score = w1 * normCount - w2 * normPerpendicular - w3 * normDist;

                if (score > bestScore) {
                    bestScore = score;
                    bestAngle = theta;
                }
            }
        }

        if (Double.isNaN(bestAngle)) return null;
        double bestAngleDeg = Math.toDegrees(bestAngle);
        if (bestAngleDeg < 0) bestAngleDeg += 360;

        return bestAngleDeg;
    }

    public void drawPoseOnPanels(FieldManager panelsField, Pose pose, String color) {
        if (!(pose == null || panelsField == null)) {
            panelsField.setStyle(color, color, 0);
            panelsField.moveCursor(pose.getX(), pose.getY());
            panelsField.circle(3);

            double headingLineLength = 3;
            double x2 = pose.getX() + Math.cos(pose.getHeading()) * headingLineLength;
            double y2 = pose.getY() + Math.sin(pose.getHeading()) * headingLineLength;

            panelsField.setStyle("black", "black", 1);
            panelsField.line(x2, y2);
        }
    }
    public double normalizeHeading360Degrees(double heading) {
        return (heading + 360) % 360;
    }

}