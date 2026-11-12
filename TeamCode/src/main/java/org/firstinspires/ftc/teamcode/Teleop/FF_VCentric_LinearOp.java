package org.firstinspires.ftc.teamcode.Teleop;

import android.util.Size;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.skeletonarmy.marrow.weaver.PathConfig;
import com.skeletonarmy.marrow.weaver.Weaver;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Annotations.DEV;
import org.firstinspires.ftc.teamcode.Robot.AutoAim;
import org.firstinspires.ftc.teamcode.Robot.Drive;
import org.firstinspires.ftc.teamcode.Robot.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Robot.ShooterSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="FF-VisionCentric-Telemetric", group="Telemetric")
@DEV
public class FF_VCentric_LinearOp extends OpMode {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private SparkFunOTOS otos;
    SparkFunOTOS.Pose2D pos;
    SparkFunOTOS.Status otosStatus;
    private AprilTagProcessor tagProcessor;
    private VisionPortal tagPortal;

    AutoAim autoAim;
    ShooterSubsystem shooter;
    IntakeSubsystem intake;
    Drive drive;
    double robotYaw;
    //FlowerSubsystem flowerSubsystem = null;

    private boolean drivetrainModeFieldCentric = false;
    boolean autoAiming = false;
    double rotate;
    boolean isBlue;
    double ANGULAR_SCALAR;
    double LINEAR_SCALAR;
    @Override
    public void init() {
        //flowerSubsystem = new FlowerSubsystem(hardwareMap);

        frontLeft = hardwareMap.get(DcMotorImplEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorImplEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorImplEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorImplEx.class, "backRight");

        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.begin();

        if (!otos.begin()) {
            telemetry.addLine("otos not started");
        }

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);

        otos.setOffset(new SparkFunOTOS.Pose2D(0, 18, 0));

        otos.calibrateImu();
        otos.resetTracking();


        //TODO: calibrate these
        otos.setAngularScalar(ANGULAR_SCALAR);
        otos.setLinearScalar(LINEAR_SCALAR);

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();


        tagPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .enableLiveView(true)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 400))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

         autoAim = new AutoAim(tagProcessor);
         shooter = new ShooterSubsystem(hardwareMap);
         intake = new IntakeSubsystem(hardwareMap);
         drive = new Drive(hardwareMap);

         shooter.MOTOR_VELOCITY = 10;
        /*
          Weaver.setConfig(new PathConfig()
                .intakeWidth(4)
                .robotWidth(12)
                .robotHeight(16.0)
                .clearance(1)
                .smoothing(true));
         */


    }

    @Override
    public void init_loop() {
        if (gamepad1.a){
            drivetrainModeFieldCentric = !drivetrainModeFieldCentric;
        }
        if (gamepad1.b){
            isBlue = !isBlue;
        }
    }

    @Override
    public void start(){
        shooter.spinUp();
        intake.spinUp();
    }

    @Override
    public void loop(){

        rotate = gamepad1.right_stick_x;

        otosStatus = otos.getStatus();
        pos = otos.getPosition();
        robotYaw = pos.h;

        telemetry.addLine("Drivetrain Data ↓");
        telemetry.addData("Front Left Pos: ", frontLeft.getCurrentPosition());
        telemetry.addData("Front Right Pos:", frontRight.getCurrentPosition());
        telemetry.addData("Back Left Pos: ", backLeft.getCurrentPosition());
        telemetry.addData("Right Back Pos: ", backRight.getCurrentPosition());
        telemetry.addData("Is robot oriented? ", drivetrainModeFieldCentric);

        telemetry.addLine("Sparkfun OTOS Data ↓");
        telemetry.addData("Otos Depth Warning: ", otosStatus.warnOpticalTracking);
        telemetry.addData("Otos Tilt Warning", otosStatus.warnTiltAngle);
        telemetry.addData("Otos Laser error: ", otosStatus.errorLsm);
        telemetry.addData("Otos Tracking Loss Error: ", otosStatus.errorPaa);
        telemetry.addData("Otos Linear Scalar: ", otos.getLinearScalar());
        telemetry.addData("Otos Angular Scalar: ", otos.getAngularScalar());

        telemetry.addLine("Pose Data ↓");
        telemetry.addData("Otos X Pos: ", pos.x);
        telemetry.addData("Otos Y Pos: ", pos.y);
        telemetry.addData("Otos θ", pos.h);
        telemetry.addData("Otos Velocity", otos.getVelocity());

        telemetry.addLine("Vision Data ↓");
        telemetry.addLine("idk I didnt do ts yet");

        telemetry.addData("Blue Alliance selected? ", isBlue);


        if (!drivetrainModeFieldCentric){
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
        if (drivetrainModeFieldCentric){
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, rotate);
        }

        if(gamepad1.a){
            autoAiming = !autoAiming;
        }

        if (autoAiming){
            drive.setRotateDegrees(autoAim.update(pos.h, isBlue ? AutoAim.Alliance.BLUE : AutoAim.Alliance.RED, rotate));
        }

        if (gamepad1.y){
            shooter.openGate();
        } else {
            shooter.closeGate();
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
