package org.firstinspires.ftc.teamcode.vision.testing;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
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
 * Java port of the Limelight-style Python runPipeline() ball detector.
 *
 * Pipeline (matches the Python version step for step):
 *   1. Convert frame to HSV
 *   2. Threshold to a binary mask via inRange(lower, upper)
 *   3. Erode / dilate to clean noise
 *   4. connectedComponentsWithStats to drop small blobs (< MIN_COMPONENT_AREA)
 *   5. findContours on the cleaned mask, filter by area (< MIN_CONTOUR_AREA)
 *   6. Draw bounding boxes on the frame
 *   7. Pack boxes into a fixed-length double[32] the same way encode_boxes() did
 *
 * NOTE ON COLOR SPACE: the Python script assumed a BGR input (OpenCV's default
 * for cv2.VideoCapture / Limelight). The FTC VisionPortal instead hands
 * processFrame() an RGBA Mat, so this converts with COLOR_RGB2HSV rather than
 * COLOR_BGR2HSV. If your HSV thresholds were tuned against BGR frames, verify
 * them against a live RGBA preview -- hue values may shift slightly (mainly
 * affects red-ish hues; your existing yellow/green-ish range should be close).
 */
public class BallDetectionProcessor implements VisionProcessor {

    // ---- HSV threshold range (same values as the Python script) ----
    private static final Scalar LOWER = new Scalar(15, 222, 201);
    private static final Scalar UPPER = new Scalar(45, 255, 255);

    // ---- Morphology kernel (7x7, all ones) ----
    private static final Mat KERNEL = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(7, 7));

    private static final int ERODE_ITERATIONS = 0;   // kept at 0 to match the Python script (effectively a no-op)
    private static final int DILATE_ITERATIONS = 1;

    private static final double MIN_COMPONENT_AREA = 1500;
    private static final double MIN_CONTOUR_AREA = 500;

    private static final int MAX_BOXES_PACKED = 15;
    private static final int MAX_DOUBLES = 32;

    // Reusable buffers so we're not allocating new Mats every frame
    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();
    private final Mat labels = new Mat();
    private final Mat stats = new Mat();
    private final Mat centroids = new Mat();
    private final Mat clean = new Mat();
    private final Mat hierarchy = new Mat();

    // Latest results, exposed for your OpMode's loop to read
    private volatile List<Rect> latestBoxes = new ArrayList<>();
    private volatile double[] latestEncoded = new double[MAX_DOUBLES];

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Buffers above are lazily sized by OpenCV on first use; nothing to do here.
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsv, LOWER, UPPER, mask);

        if (ERODE_ITERATIONS > 0) {
            Imgproc.erode(mask, mask, KERNEL, new Point(-1, -1), ERODE_ITERATIONS);
        }
        Imgproc.dilate(mask, mask, KERNEL, new Point(-1, -1), DILATE_ITERATIONS);

        int n = Imgproc.connectedComponentsWithStats(mask, labels, stats, centroids, 8, CvType.CV_32S);

        clean.create(mask.size(), CvType.CV_8UC1);
        clean.setTo(new Scalar(0));

        Mat componentMask = new Mat();
        for (int i = 1; i < n; i++) {
            double area = stats.get(i, Imgproc.CC_STAT_AREA)[0];
            if (area >= MIN_COMPONENT_AREA) {
                Core.compare(labels, new Scalar(i), componentMask, Core.CMP_EQ);
                clean.setTo(new Scalar(255), componentMask);
            }
        }
        componentMask.release();

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(clean, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        List<Rect> boxes = new ArrayList<>();

        for (MatOfPoint cnt : contours) {
            double area = Imgproc.contourArea(cnt);
            if (area >= MIN_CONTOUR_AREA) {
                Rect box = Imgproc.boundingRect(cnt);
                boxes.add(box);

                Imgproc.rectangle(
                        frame,
                        new Point(box.x, box.y),
                        new Point(box.x + box.width, box.y + box.height),
                        new Scalar(255, 255, 0), // yellow
                        5
                );
            }
            cnt.release();
        }

        double[] encoded = encodeBoxes(boxes);

        latestBoxes = boxes;
        latestEncoded = encoded;

        return encoded;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Boxes are already drawn directly onto the frame in processFrame(),
        // so there's nothing extra needed on the overlay canvas.
    }

    /**
     * Packs bounding boxes into a fixed-length double[32], mirroring the
     * Python encode_boxes(): count first, then each box packed as two
     * doubles ((x << 11) | w) and ((y << 10) | h), zero-padded to length 32.
     */
    private double[] encodeBoxes(List<Rect> boxes) {
        int count = Math.min(boxes.size(), MAX_BOXES_PACKED);
        double[] output = new double[MAX_DOUBLES]; // defaults to all 0.0
        int idx = 0;
        output[idx++] = (double) count;

        for (int i = 0; i < count; i++) {
            Rect r = boxes.get(i);
            int x = clamp(r.x, 0, 2047);
            int y = clamp(r.y, 0, 1023);
            int w = clamp(r.width, 0, 2047);
            int h = clamp(r.height, 0, 1023);

            output[idx++] = (double) ((x << 11) | w);
            output[idx++] = (double) ((y << 10) | h);
        }
        return output;
    }

    private static int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(value, max));
    }

    /** Latest detected bounding boxes, for use in your OpMode's loop. */
    public List<Rect> getLatestBoxes() {
        return latestBoxes;
    }

    /** Latest packed double[32], if you want the same encoding as the Python version. */
    public double[] getLatestEncoded() {
        return latestEncoded;
    }
}