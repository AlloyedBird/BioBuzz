package org.firstinspires.ftc.teamcode.Teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Annotations.TESTING;
import org.firstinspires.ftc.teamcode.Robot.Vision.VisionCircularProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import java.util.List;

@TeleOp(name = "AprilTag Telemetry OpMode", group = "Telemetric")
@TESTING
public class PF_VCentric_NLinearTest extends LinearOpMode {

    // Declare VisionPortal and AprilTag Processor
    private AprilTagProcessor aprilTag;
    private VisionPortal tagPortal;
    private VisionPortal circlePortal;

    private VisionCircularProcessor circler;

    @Override
    public void runOpMode() {

        // 1. Initialize the AprilTag Processor
        aprilTag = new AprilTagProcessor.Builder()
                // You can add custom calibrations or tag libraries here if needed
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .build();

        circler = new VisionCircularProcessor();

        // 2. Initialize the VisionPortal
        // Note: Ensure your webcam name exactly matches the robot configuration
        tagPortal = new VisionPortal.Builder()
                .setCameraResolution(new Size(640, 400))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        // Custom circle processor type shi
        circlePortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "colorCamera"))
                .build();



        telemetry.addLine("VisionPortal initialized. Waiting for start...");
        telemetry.update();

        // Wait for the driver to press START
        waitForStart();

        // 3. Main Loop
        while (opModeIsActive()) {

            // Get the list of detections from the processor
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("Total Tags Detected", currentDetections.size());

            // 4. Iterate through detections
            for (AprilTagDetection detection : currentDetections) {

                // IMPORTANT (SDK 12.0+): Check if the detection is a single tag or a cluster
                if (detection instanceof AprilTagSingleDetection) {

                    // Cast to single detection to access .id and .metadata
                    AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;

                    // The FTC SDK only calculates pose if the tag size/metadata is known
                    if (singleDet.metadata != null) {
                        telemetry.addLine(String.format("\n==== Single Tag (ID %d) ====", singleDet.id));
                        telemetry.addLine(String.format("Name: %s", singleDet.metadata.name));

                        // Translation (XYZ in inches)
                        telemetry.addLine(String.format("XYZ (in): %6.2f, %6.2f, %6.2f",
                                singleDet.ftcPose.x, singleDet.ftcPose.y, singleDet.ftcPose.z));

                        // Rotation (Pitch, Roll, Yaw in degrees)
                        telemetry.addLine(String.format("PRY (deg): %6.2f, %6.2f, %6.2f",
                                singleDet.ftcPose.pitch, singleDet.ftcPose.roll, singleDet.ftcPose.yaw));

                        // Polar Coordinates (Range, Bearing, Elevation)
                        telemetry.addLine(String.format("RBE (in/deg): %6.2f, %6.2f, %6.2f",
                                singleDet.ftcPose.range, singleDet.ftcPose.bearing, singleDet.ftcPose.elevation));
                    } else {
                        // A tag was found, but it is not a part of the official game library
                        telemetry.addLine(String.format("\n==== Unknown Single Tag (ID %d) ====", singleDet.id));
                    }

                } else if (detection instanceof AprilTagClusterDetection) {

                    // Cast to cluster detection if multiple tags are grouped together
                    AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;

                    if (clusterDet.metadata != null) {
                        telemetry.addLine(String.format("\n==== Cluster: %s ====", clusterDet.metadata.name));
                        telemetry.addLine(String.format("Visibility: %d%%", clusterDet.percentClusterFound));

                        telemetry.addLine(String.format("XYZ (in): %6.2f, %6.2f, %6.2f",
                                clusterDet.ftcPose.x, clusterDet.ftcPose.y, clusterDet.ftcPose.z));
                        telemetry.addLine(String.format("PRY (deg): %6.2f, %6.2f, %6.2f",
                                clusterDet.ftcPose.pitch, clusterDet.ftcPose.roll, clusterDet.ftcPose.yaw));
                    }
                }
            }

            // Push the telemetry data to the Driver Station
            telemetry.update();

            // Briefly sleep to save CPU resources
            sleep(20);
        }

        // Clean up when opmode is stopped
        if (tagPortal != null) {
            tagPortal.close();
        }
    }
}