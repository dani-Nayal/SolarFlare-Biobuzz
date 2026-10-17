package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class PollenDetector implements VisionProcessor {

    // Same values as your Python script (OpenCV hue range is 0-179 in both)
    private static final Scalar LOWER = new Scalar(10, 69, 162);
    private static final Scalar UPPER = new Scalar(30, 189, 255);

    private static final int MIN_AREA = 40;
    private static final int KEEP_LARGEST = 40;

    private static final Scalar BOX_COLOR = new Scalar(255, 255, 0, 255); // yellow (frame is RGB)

    // Set true to overlay the raw HSV mask on the stream for tuning
    public volatile boolean showMask = false;

    // Reused every frame to avoid allocating in the camera loop
    private final Mat blurred = new Mat();
    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();
    private final Mat labels = new Mat();
    private final Mat stats = new Mat();
    private final Mat centroids = new Mat();
    private final Mat maskRgba = new Mat();

    // Written on the camera thread, read from your OpMode thread
    private volatile List<Rect> latestBoxes = new ArrayList<>();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.GaussianBlur(frame, blurred, new Size(5, 5), 0);
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV); // FTC frames are RGB, not BGR
        Core.inRange(hsv, LOWER, UPPER, mask);

        int n = Imgproc.connectedComponentsWithStats(
                mask, labels, stats, centroids, 8, CvType.CV_32S);

        // stats is n x 5 (CV_32S): LEFT, TOP, WIDTH, HEIGHT, AREA
        int[] data = new int[n * 5];
        stats.get(0, 0, data);

        List<int[]> comps = new ArrayList<>();
        for (int i = 1; i < n; i++) { // skip background (label 0)
            int area = data[i * 5 + Imgproc.CC_STAT_AREA];
            if (area >= MIN_AREA) {
                comps.add(new int[]{
                        data[i * 5 + Imgproc.CC_STAT_LEFT],
                        data[i * 5 + Imgproc.CC_STAT_TOP],
                        data[i * 5 + Imgproc.CC_STAT_WIDTH],
                        data[i * 5 + Imgproc.CC_STAT_HEIGHT],
                        area});
            }
        }

        // largest first, keep at most KEEP_LARGEST
        Collections.sort(comps, (a, b) -> Integer.compare(b[4], a[4]));
        if (comps.size() > KEEP_LARGEST) {
            comps = comps.subList(0, KEEP_LARGEST);
        }

        List<Rect> boxes = new ArrayList<>();
        for (int[] c : comps) {
            int x = c[0], y = c[1], w = c[2], h = c[3];

            // make the box square, centered on the blob
            int s = Math.max(w, h);
            x -= (s - w) / 2;
            y -= (s - h) / 2;

            Rect box = new Rect(x, y, s, s);
            boxes.add(box);
            Imgproc.rectangle(frame, box.tl(), box.br(), BOX_COLOR, 3);
        }

        if (showMask) {
            Imgproc.cvtColor(mask, maskRgba, Imgproc.COLOR_GRAY2RGBA);
            Core.addWeighted(frame, 0.5, maskRgba, 0.5, 0, frame);
        }

        latestBoxes = boxes;
        return boxes; // also delivered to onDrawFrame as userContext
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        // Boxes are already drawn on the frame in processFrame, so nothing to do here
    }

    public List<Rect> getBoxes() {
        return latestBoxes;
    }
}