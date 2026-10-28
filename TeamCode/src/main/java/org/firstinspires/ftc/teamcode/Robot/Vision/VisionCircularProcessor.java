package org.firstinspires.ftc.teamcode.Robot.Vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class VisionCircularProcessor implements VisionProcessor {

    // ── HSV thresholds for yellow ──────────────────────────────────────────
    private static final Scalar YELLOW_LOW  = new Scalar(20, 100, 100);
    private static final Scalar YELLOW_HIGH = new Scalar(35, 255, 255);

    // ── Hough params (tune these on the actual field) ──────────────────────
    private static final double DP         = 1.2;  // inverse resolution ratio
    private static final double MIN_DIST   = 50;   // min px between circle centers
    private static final double PARAM1     = 100;  // Canny high threshold
    private static final double PARAM2     = 30;   // accumulator threshold — lower = more circles + more false positives
    private static final int    MIN_RADIUS = 10;
    private static final int    MAX_RADIUS = 150;

    // ── Reusable mats (allocated once, reused every frame) ─────────────────
    private final Mat hsv     = new Mat();
    private final Mat mask    = new Mat();
    private final Mat circles = new Mat();

    // ── Results ────────────────────────────────────────────────────────────
    // Each float[] is { centerX, centerY, radius } in frame pixels
    private List<float[]> detectedCircles = new ArrayList<>();
    private final Object lock = new Object();

    // ── Debug paint ────────────────────────────────────────────────────────
    private final Paint outlinePaint;
    private final Paint centerPaint;

    public VisionCircularProcessor() {
        outlinePaint = new Paint();
        outlinePaint.setColor(Color.YELLOW);
        outlinePaint.setStyle(Paint.Style.STROKE);
        outlinePaint.setStrokeWidth(4);

        centerPaint = new Paint();
        centerPaint.setColor(Color.RED);
        centerPaint.setStyle(Paint.Style.FILL);
    }

    // ── VisionProcessor ────────────────────────────────────────────────────

    @Override
    public void init(int width, int height, CameraCalibration calibration) { }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // RGB → HSV
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);

        // Isolate yellow
        Core.inRange(hsv, YELLOW_LOW, YELLOW_HIGH, mask);

        // Slight blur to smooth noise before Hough
        Imgproc.GaussianBlur(mask, mask, new Size(5, 5), 2);

        // Detect circles
        Imgproc.HoughCircles(mask, circles, Imgproc.HOUGH_GRADIENT,
                DP, MIN_DIST, PARAM1, PARAM2, MIN_RADIUS, MAX_RADIUS);

        // Parse into list
        List<float[]> found = new ArrayList<>();
        for (int i = 0; i < circles.cols(); i++) {
            double[] c = circles.get(0, i);
            if (c != null) {
                found.add(new float[]{ (float) c[0], (float) c[1], (float) c[2] });
            }
        }

        synchronized (lock) {
            detectedCircles = found;
        }

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        synchronized (lock) {
            for (float[] c : detectedCircles) {
                float cx = c[0] * scaleBmpPxToCanvasPx;
                float cy = c[1] * scaleBmpPxToCanvasPx;
                float r  = c[2] * scaleBmpPxToCanvasPx;

                canvas.drawCircle(cx, cy, r, outlinePaint);
                canvas.drawCircle(cx, cy, 8 * scaleCanvasDensity, centerPaint);
            }
        }
    }

    // ── Public API for OpMode ──────────────────────────────────────────────

    /** All detected circles as { x, y, radius } */
    public List<float[]> getDetectedCircles() {
        synchronized (lock) {
            return new ArrayList<>(detectedCircles);
        }
    }

    /** Largest circle by radius, or null if none found */
    public float[] getLargestCircle() {
        synchronized (lock) {
            float[] largest = null;
            for (float[] c : detectedCircles) {
                if (largest == null || c[2] > largest[2]) {
                    largest = c;
                }
            }
            return largest;
        }
    }

    /** How many circles are currently detected */
    public int getCircleCount() {
        synchronized (lock) {
            return detectedCircles.size();
        }
    }
}