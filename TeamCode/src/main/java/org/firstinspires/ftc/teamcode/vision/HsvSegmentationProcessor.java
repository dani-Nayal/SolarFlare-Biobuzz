package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * Basic HSV segmentation as an FTC VisionProcessor.
 * Pipeline: blur -> HSV -> inRange -> open/close -> contours -> area filter -> draw.
 * OpenCV HSV ranges: H 0-179, S 0-255, V 0-255.
 *
 * Usage:
 *   HsvSegmentationProcessor hsv = new HsvSegmentationProcessor();
 *   VisionPortal portal = new VisionPortal.Builder()
 *       .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
 *       .addProcessor(hsv)
 *       .build();
 *   // in loop: hsv.getDetections()
 */
public class HsvSegmentationProcessor implements VisionProcessor {

    // Tunable thresholds (default: yellow-ish). Public volatile so they can be tweaked live.
    public volatile Scalar lower = new Scalar(20, 100, 100);
    public volatile Scalar upper = new Scalar(35, 255, 255);
    public volatile double minArea = 150;

    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();
    private final Mat blurred = new Mat();
    private final Mat kernel =
            Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));

    private volatile List<Rect> detections = new ArrayList<>();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Nothing to set up for this simple pipeline.
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // FTC delivers RGBA frames.
        Imgproc.GaussianBlur(frame, blurred, new Size(7, 7), 0);
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);

        org.opencv.core.Core.inRange(hsv, lower, upper, mask);

        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(),
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        List<Rect> found = new ArrayList<>();
        for (MatOfPoint c : contours) {
            if (Imgproc.contourArea(c) < minArea) continue;
            Rect r = Imgproc.boundingRect(c);
            found.add(r);
            // Draw straight onto the frame so it shows in the Driver Station preview.
            Imgproc.rectangle(frame, r, new Scalar(0, 255, 0), 2);
            Imgproc.circle(frame,
                    new Point(r.x + r.width / 2.0, r.y + r.height / 2.0),
                    3, new Scalar(255, 0, 0), -1);
            c.release();
        }
        detections = found;
        return null; // nothing extra needed in onDrawFrame
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        // Overlays were already drawn into the Mat above.
    }

    public List<Rect> getDetections() {
        return detections;
    }
}