package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Teleop trust", group = "Teleops")
public class Teleop extends OpMode {
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private IMU imu;
    

    @Override
    public void init() {
        leftFront  = hardwareMap.get(DcMotorImplEx.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotorImplEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorImplEx.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotorImplEx.class, "rightBack");

        // Set brake mode so motors stop cleanly on stick release
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
            new com.qualcomm.hardware.rev.RevHubOrientationOnRobot(
                com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection.UP,
                com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        ));

        // Standard mecanum: reverse left side only
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        driveFieldRelative(
            -gamepad1.left_stick_y,
             gamepad1.left_stick_x,
             gamepad1.right_stick_x
        );
    }

    private void driveFieldRelative(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newRight   = r * Math.cos(theta);
        drive(newForward, newRight, rotate);
    }

    private void drive(double forward, double right, double rotate) {
        double frontLeftPower  = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backLeftPower   = forward - right + rotate;
        double backRightPower  = forward + right - rotate;

        double maxPower = Math.max(1.0, Math.max(
            Math.max(Math.abs(frontLeftPower),  Math.abs(frontRightPower)),
            Math.max(Math.abs(backLeftPower),   Math.abs(backRightPower))
        ));

        leftFront.setPower(frontLeftPower  / maxPower);
        rightFront.setPower(frontRightPower / maxPower);
        leftBack.setPower(backLeftPower   / maxPower);
        rightBack.setPower(backRightPower  / maxPower);
    }
}
