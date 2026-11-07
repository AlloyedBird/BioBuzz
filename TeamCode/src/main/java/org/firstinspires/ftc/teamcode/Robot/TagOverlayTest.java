package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.Vision.AprilTagOverlayProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "Tag Overlay test ", group = "test")
public class TagOverlayTest extends LinearOpMode {
    private AprilTagProcessor aprilTag;
    private VisionPortal tagPortal;

    @Override
    public void runOpMode() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setDrawAxes(true)
                .build();


        tagPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new Size(640,400))
                .build();

        waitForStart();

        while (opModeIsActive()) {
            // your teleop loop here
        }

        tagPortal.close();
    }
}

