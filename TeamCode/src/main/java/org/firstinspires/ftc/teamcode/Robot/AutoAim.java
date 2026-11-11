package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AutoAim {
    public enum Alliance { RED, BLUE }

    // Shooter points 90° left of camera forward. Positive bearing = target is left of camera,
    // so the shooter is lined up when bearing == +90.
    private static final double SHOOTER_OFFSET_DEG = 0;
    // Drive code: positive rotate = clockwise. OTOS heading: positive = counterclockwise.
    private static final double TURN_SIGN = -1.0;
    private static final double DEADBAND_DEG = 1.0;

    public final AprilTagProcessor aprilTag;

    private boolean hasTarget = false;
    private double targetHeadingDeg = 0;

    public AutoAim(AprilTagProcessor aprilTag) {
        this.aprilTag = aprilTag;
    }

    public boolean hasTarget() { return hasTarget; }

    //This method returns the angle to turn to to properly aim the shooter
       public double update(double headingDeg, Alliance alliance, double manualRotate) {
        AprilTagClusterDetection best = findBestCluster(alliance);

        if (best != null) {
            // Convert the camera-relative bearing into a field heading using OTOS,
            // so we can keep aiming for a moment if the cluster drops out of view.
            double bearingError = best.ftcPose.bearing - SHOOTER_OFFSET_DEG;
            targetHeadingDeg = headingDeg + bearingError;
            hasTarget = true;
        }

        if (!hasTarget) return manualRotate;

        return AngleUnit.normalizeDegrees(targetHeadingDeg - headingDeg);
    }

    private AprilTagClusterDetection findBestCluster(Alliance alliance) {
        String prefix = (alliance == Alliance.RED) ? "R" : "B";
        AprilTagClusterDetection best = null;

        for (AprilTagDetection det : aprilTag.getDetections()) {
            if (!(det instanceof AprilTagClusterDetection)) continue;
            AprilTagClusterDetection c = (AprilTagClusterDetection) det;

            if (!c.metadata.name.startsWith(prefix)) continue;       // wrong alliance
            if (Math.abs(c.ftcPose.roll) >= 90) continue;            // upside-down = not scorable

            if (best == null
                    || c.percentClusterFound > best.percentClusterFound
                    || (c.percentClusterFound == best.percentClusterFound
                    && c.ftcPose.range < best.ftcPose.range)) {
                best = c;
            }
        }
        return best;
    }
}