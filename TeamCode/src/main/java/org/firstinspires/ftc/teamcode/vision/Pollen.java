package org.firstinspires.ftc.teamcode.vision;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.math.BigDecimal;
import java.math.BigInteger;
import java.util.ArrayList;
import java.util.List;

public class Pollen {
    public final double DIAMETER_INCHES = 2.85;
    public final double fx = 1218.145;
    public final double fy = 1219.418;
    public final double cx = 621.829;
    public final double cy = 500.362;
    public double x;
    public double y;
    Corners corners;
    public double tx;
    public double ty;
    public boolean isValid = true;
    public double concentration = 1;
    public Pollen(LLResultTypes.ColorResult detection, Vision.CAMERA_ORIENTATION cameraOrientation, Pose botPose, Pose3D cameraPoseOnRobot){
        if (detection == null) return;

        this.corners = new Corners(detection.getTargetCorners(), cameraOrientation);

        double width = Math.abs(corners.leftBottomX - corners.rightBottomX);
        double height = Math.abs(corners.rightBottomY - corners.rightTopY);

        if (0.7 < (height / width) && (height / width) < 1.3){
            if (height > width){
                width += height - width;
            }
            else if (width > height){
                height += width - height;
            }
        }

        double centerX = (corners.leftBottomX + corners.rightBottomX) / 2;
        double centerY = (corners.leftBottomY + corners.leftTopY) / 2;

        this.tx = Math.toDegrees(Math.atan((centerX - cx) / fx));
        this.ty = Math.toDegrees(Math.atan((cy - centerY) / fy));

        double verticalAngleDeg = ty + 90 + cameraPoseOnRobot.getOrientation().getPitch(AngleUnit.DEGREES);

        double depth = ((cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).z - (DIAMETER_INCHES / 2)) * Math.tan(Math.toRadians(verticalAngleDeg)));

        double horizontal = -(depth * Math.tan(Math.toRadians(tx + cameraPoseOnRobot.getOrientation().getYaw(AngleUnit.DEGREES))));

        // relativeY depends on relativeX being relative to camera, not center of robot
        depth += cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).x;
        horizontal += cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).y;

        double botPoseX = botPose.getX();
        double botPoseY = botPose.getY();
        double theta = botPose.getHeading();

        double cos = Math.cos(theta);
        double sin = Math.sin(theta);

        this.x = botPoseX + depth * cos - (horizontal) * sin;
        this.y = botPoseY + depth * sin + (horizontal) * cos;

        if (!(x >= 0 && x <= 144 && y >= 0 && y <= 144)){
            this.isValid = false;
            return;
        }

        double area = height * width;
        double predictedArea = Math.pow((DIAMETER_INCHES * fy) / depth, 2);

        double ratio = area / predictedArea;

        if (ratio > 1.3){
            this.concentration = Math.ceil(ratio);
        }
    }

    public Pollen(LLResultTypes.DetectorResult detection, Vision.CAMERA_ORIENTATION cameraOrientation, Pose botPose, Pose3D cameraPoseOnRobot){
        if (detection == null) return;

        this.corners = new Corners(detection.getTargetCorners(), cameraOrientation);

        double centerX = (corners.leftBottomX + corners.rightBottomX) / 2;
        double centerY = (corners.leftBottomY + corners.leftTopY) / 2;

        this.tx = Math.toDegrees(Math.atan((centerX - cx) / fx));
        this.ty = Math.toDegrees(Math.atan((cy - centerY) / fy));

        double verticalAngleDeg = ty + 90 + cameraPoseOnRobot.getOrientation().getPitch(AngleUnit.DEGREES);

        double depth = ((cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).z - (DIAMETER_INCHES / 2)) * Math.tan(Math.toRadians(verticalAngleDeg)));

        double horizontal = -(depth * Math.tan(Math.toRadians(tx + cameraPoseOnRobot.getOrientation().getYaw(AngleUnit.DEGREES))));

        // relativeY depends on relativeX being relative to camera, not center of robot
        depth += cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).x;
        horizontal += cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).y;

        double botPoseX = botPose.getX();
        double botPoseY = botPose.getY();
        double theta = botPose.getHeading();

        double cos = Math.cos(theta);
        double sin = Math.sin(theta);

        this.x = botPoseX + depth * cos - (horizontal) * sin;
        this.y = botPoseY + depth * sin + (horizontal) * cos;

        if (!(x >= 0 && x <= 144 && y >= 0 && y <= 144)){
            this.isValid = false;
        }
    }

    public Pollen(double x, double y, double w, double h, Pose botPose, Pose3D cameraPoseOnRobot){

        if (0.7 < (h / w) && (h / w) < 1.3){
            if (h > w){
                w += h - w;
            }
            else if (w > h){
                h += w - h;
            }
        }

        double centerX = x + (w / 2);
        double centerY = y + (h / 2);

        this.tx = Math.toDegrees(Math.atan((centerX - cx) / fx));
        this.ty = Math.toDegrees(Math.atan((cy - centerY) / fy));

        double verticalAngleDeg = ty + 90 + cameraPoseOnRobot.getOrientation().getPitch(AngleUnit.DEGREES);

        double depth = ((cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).z - (DIAMETER_INCHES / 2)) * Math.tan(Math.toRadians(verticalAngleDeg)));

        double horizontal = -(depth * Math.tan(Math.toRadians(tx + cameraPoseOnRobot.getOrientation().getYaw(AngleUnit.DEGREES))));

        // relativeY depends on relativeX being relative to camera, not center of robot
        depth += cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).x;
        horizontal += cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).y;

        double botPoseX = botPose.getX();
        double botPoseY = botPose.getY();
        double theta = botPose.getHeading();

        double cos = Math.cos(theta);
        double sin = Math.sin(theta);

        this.x = botPoseX + depth * cos - (horizontal) * sin;
        this.y = botPoseY + depth * sin + (horizontal) * cos;

        if (!(x >= 0 && x <= 144 && y >= 0 && y <= 144)){
            this.isValid = false;
            return;
        }

        double area = h * w;
        double predictedArea = Math.pow((DIAMETER_INCHES * fy) / depth, 2);

        double ratio = area / predictedArea;

        if (ratio > 1.3){
            this.concentration = Math.ceil(ratio);
        }
    }

    class Corners {
        double leftTopX;
        double leftTopY;
        double rightTopX;
        double rightTopY;
        double leftBottomX;
        double leftBottomY;
        double rightBottomX;
        double rightBottomY;

        public Corners(List<List<Double>> cornersLL, Vision.CAMERA_ORIENTATION cameraOrientation){
            switch (cameraOrientation){
                case NORMAL:
                    leftTopX = cornersLL.get(0).get(0);
                    leftTopY = cornersLL.get(0).get(1);
                    rightTopX = cornersLL.get(1).get(0);
                    rightTopY = cornersLL.get(1).get(1);
                    rightBottomX = cornersLL.get(2).get(0);
                    rightBottomY = cornersLL.get(2).get(1);
                    leftBottomX = cornersLL.get(3).get(0);
                    leftBottomY = cornersLL.get(3).get(1);
                    break;
                case COUNTER_CLOCKWISE_90:
                    leftTopX = cornersLL.get(1).get(0);
                    leftTopY = cornersLL.get(1).get(1);
                    rightTopX = cornersLL.get(2).get(0);
                    rightTopY = cornersLL.get(2).get(1);
                    rightBottomX = cornersLL.get(3).get(0);
                    rightBottomY = cornersLL.get(3).get(1);
                    leftBottomX = cornersLL.get(0).get(0);
                    leftBottomY = cornersLL.get(0).get(1);
                    break;
                case CLOCKWISE_90:
                    leftTopX = cornersLL.get(3).get(0);
                    leftTopY = cornersLL.get(3).get(1);
                    rightTopX = cornersLL.get(0).get(0);
                    rightTopY = cornersLL.get(0).get(1);
                    rightBottomX = cornersLL.get(1).get(0);
                    rightBottomY = cornersLL.get(1).get(1);
                    leftBottomX = cornersLL.get(2).get(0);
                    leftBottomY = cornersLL.get(2).get(1);
                    break;
                case UPSIDE_DOWN:
                    leftTopX = cornersLL.get(2).get(0);
                    leftTopY = cornersLL.get(2).get(1);
                    rightTopX = cornersLL.get(3).get(0);
                    rightTopY = cornersLL.get(3).get(1);
                    rightBottomX = cornersLL.get(0).get(0);
                    rightBottomY = cornersLL.get(0).get(1);
                    leftBottomX = cornersLL.get(1).get(0);
                    leftBottomY = cornersLL.get(1).get(1);
            }
        }
    }

    public static List<int[]> decodeBoxes(double[] data) {
        List<int[]> boxes = new ArrayList<>();
        int numBoxes = (int) Math.round(data[0]);

        for (int i = 0; i < numBoxes; i++) {
            int base = 1 + i * 2;
            int xw = (int) Math.round(data[base]);
            int yh = (int) Math.round(data[base + 1]);

            int x = (xw >> 11) & 0x7FF;
            int w = xw & 0x7FF;
            int y = (yh >> 10) & 0x3FF;
            int h = yh & 0x3FF;

            boxes.add(new int[]{x, y, w, h});
        }
        return boxes;
    }


    private static int getBits(
            BigInteger value,
            int shift,
            int bits) {

        BigInteger mask =
                BigInteger.ONE.shiftLeft(bits)
                        .subtract(BigInteger.ONE);

        return value
                .shiftRight(shift)
                .and(mask)
                .intValue();
    }
}
