package org.firstinspires.ftc.teamcode.Robot.Vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

public class AprilTagOverlayProcessor implements VisionProcessor {

    private AprilTagProcessor aprilTag;
    private List<AprilTagDetection> detections = new ArrayList<>();
    private final Object lock = new Object();

    private Paint linePaint;
    private Paint textPaint;
    private Paint dotPaint;

    public AprilTagOverlayProcessor() {
        linePaint = new Paint();
        linePaint.setColor(Color.GREEN);
        linePaint.setStrokeWidth(3);
        linePaint.setStyle(Paint.Style.STROKE);

        textPaint = new Paint();
        textPaint.setColor(Color.WHITE);
        textPaint.setTextSize(28);

        dotPaint = new Paint();
        dotPaint.setColor(Color.RED);
        dotPaint.setStyle(Paint.Style.FILL);
    }

    public void setAprilTagProcessor(AprilTagProcessor processor) {
        this.aprilTag = processor;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) { }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (aprilTag == null) return null;

        synchronized (lock) {
            detections = aprilTag.getDetections();
        }

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {

        float centerX = onscreenWidth / 2f;
        float centerY = onscreenHeight / 2f;

        // draw crosshair at camera center
        canvas.drawCircle(centerX, centerY, 10 * scaleCanvasDensity, dotPaint);

        synchronized (lock) {
            for (AprilTagDetection detection : detections) {
                if (detection.ftcPose == null) continue;

                // tag center in canvas coords
                // rough screen position from bearing/elevation
                // adjust FOV values to match your camera
                float hFOV = 60f; // horizontal field of view in degrees
                float vFOV = 45f; // vertical field of view in degrees

                float tagX = centerX + (float)(detection.ftcPose.bearing / (hFOV / 2f)) * (onscreenWidth / 2f);
                float tagY = centerY - (float)(detection.ftcPose.elevation / (vFOV / 2f)) * (onscreenHeight / 2f);

                // line from camera center to tag
                canvas.drawLine(centerX, centerY, tagX, tagY, linePaint);

                // dot on tag center
                canvas.drawCircle(tagX, tagY, 12 * scaleCanvasDensity, dotPaint);


                //labeling the tags
                String idOrName = "???";

                if (detection instanceof AprilTagSingleDetection) {
                    AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
                    idOrName = "ID: " + single.id;
                } else if (detection instanceof AprilTagClusterDetection) {
                    AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                    if (cluster.metadata != null) {
                        idOrName = cluster.metadata.name;
                    }
                }

                // Now format your label cleanly using that dynamic string
                String label = String.format(
                        "%s  dist:%.1fin  bear:%.1f°  elev:%.1f°",
                        idOrName,
                        detection.ftcPose.range,
                        detection.ftcPose.bearing,
                        detection.ftcPose.elevation
                );


                canvas.drawText(label, tagX - 80, tagY - 20 * scaleCanvasDensity, textPaint);
            }
        }
    }

    public List<AprilTagDetection> getDetections() {
        synchronized (lock) {
            return new ArrayList<>(detections);
        }
    }
}