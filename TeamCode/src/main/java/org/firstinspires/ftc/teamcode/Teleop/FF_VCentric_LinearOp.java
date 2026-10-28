package org.firstinspires.ftc.teamcode.Teleop;

import android.util.Size;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Annotations.DEV;
import org.firstinspires.ftc.teamcode.Robot.Vision.AprilTagOverlayProcessor;
import org.firstinspires.ftc.teamcode.Robot.Vision.VisionCircularProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@TeleOp(name="FF-Telemetric", group="Telemetric")
@DEV
public class FF_VCentric_LinearOp extends OpMode {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private SparkFunOTOS otos;
    SparkFunOTOS.Pose2D pos;
    SparkFunOTOS.Status otosStatus;
    private AprilTagOverlayProcessor tagOverlayProcessor;
    private VisionCircularProcessor circularProcessor;
    private VisionPortal tagPortal;
    private VisionPortal circlePortal;
    private List detectedCircles;

    List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

    double robotYaw;
    //FlowerSubsystem flowerSubsystem = null;

    private boolean drivetrainModeFieldCentric = false;

    @Override
    public void init() {
        //flowerSubsystem = new FlowerSubsystem(hardwareMap);

        frontLeft = hardwareMap.get(DcMotorImplEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorImplEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorImplEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorImplEx.class, "backRight");


        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.begin();

        if (!otos.begin()) {
            telemetry.addLine("otos not started");
        }

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);

        //otos.setOffset(new SparkFunOTOS.Pose2D(xOffset, yOffset, hOffset));

        otos.calibrateImu();
        otos.resetTracking();

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        tagOverlayProcessor = new AprilTagOverlayProcessor();
        circularProcessor = new VisionCircularProcessor();

        tagPortal = new VisionPortal.Builder()
                .addProcessor(tagOverlayProcessor)
                .enableLiveView(true)
                .setCamera(hardwareMap.get(WebcamName.class, "tagCamera"))
                .setCameraResolution(new Size(640, 400))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        circlePortal = new VisionPortal.Builder()
                .addProcessor(circularProcessor)
                .enableLiveView(true)
                .setCamera(hardwareMap.get(WebcamName.class, "colorCamera"))
                .build();

    }

    @Override
    public void init_loop() {
        if (gamepad1.a){
            drivetrainModeFieldCentric = !drivetrainModeFieldCentric;
        }
    }

    @Override
    public void loop(){
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        detectedCircles = circularProcessor.getDetectedCircles();

        robotYaw = pos.h;
        otosStatus = otos.getStatus();
        pos = otos.getPosition();


        telemetry.addLine("Drivetrain Data ↓");
        telemetry.addData("Front Left Pos: ", frontLeft.getCurrentPosition());
        telemetry.addData("Front Right Pos:", frontRight.getCurrentPosition());
        telemetry.addData("Back Left Pos: ", backLeft.getCurrentPosition());
        telemetry.addData("Right Back Pos: ", backRight.getCurrentPosition());
        telemetry.addData("Is robot oriented? ", drivetrainModeFieldCentric);

        telemetry.addLine("Sparkfun Data ↓");
        telemetry.addData("Otos Depth Error: ", otosStatus.warnOpticalTracking);
        telemetry.addData("Otos Tilt Error", otosStatus.warnTiltAngle);
        telemetry.addData("Otos Laser error: ", otosStatus.errorLsm);
        telemetry.addData("Otos Tracking Loss Error: ", otosStatus.errorPaa);

        telemetry.addLine("Pose Data ↓");
        telemetry.addData("Otos X Pos: ", pos.x);
        telemetry.addData("Otos Y Pos: ", pos.y);
        telemetry.addData("Otos θ", pos.h);
        telemetry.addLine("Vision Data ↓");
        telemetry.addData("Total Number of Detections", detectedCircles.size());
        if (!detectedCircles.isEmpty() && detectedCircles.size() >= 3) {
            telemetry.addData("Detected Circles", "CenterX: %.1f px, CenterY %.1f px, Radius %.1f px",
                    detectedCircles.get(0), detectedCircles.get(1), detectedCircles.get(2));
        } else {
            telemetry.addLine("Incomplete Vision Data...");
        }

        if (drivetrainModeFieldCentric){
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
        if (!drivetrainModeFieldCentric){
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        telemetry.update();
    }

    public void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        frontLeft.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRight.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeft.setPower(maxSpeed * (backLeftPower / maxPower));
        backRight.setPower(maxSpeed * (backRightPower / maxPower));
    }

    private void driveFieldRelative(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);
        theta = AngleUnit.normalizeRadians(theta -
                Math.toRadians(pos.h));

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);
        drive(newForward, newRight, rotate);
    }
}
