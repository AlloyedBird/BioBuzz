package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OTOSConfig;
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    private static MecanumConfig drivetrainConfig;
    private static OTOSConfig localizerConfig;
    private static ForesightConfig foresightConfig;

    public static void reset() {
        drivetrainConfig = null;
        localizerConfig = null;
        foresightConfig = null;
    }

    private static void init() {
        drivetrainConfig = new MecanumConfig(c -> {
            c.frontLeftName.set("frontLeft");
            c.frontRightName.set("frontRight");
            c.backLeftName.set("backLeft");
            c.backRightName.set("backRight");
            c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        });

        localizerConfig = new OTOSConfig(c -> {
            c.name.set("otos");
            c.linearScalar.set(27.743573333333334);
            c.angularScalar.set(1.0256394205112143);
            c.offset.set(new Pose(1.0933463029035433, -6.397878091166339));
            c.linearUnit.set(DistanceUnit.INCH);
        });

        foresightConfig = new ForesightConfig(fc -> {
            // Required controllers
            fc.headingFeedback.set(Controller.pid(2.0, 0.0, 0.1));
            fc.forwardTranslational.set(Controller.pid(0.1, 0.0, 0.0));
            fc.strafeTranslational.set(Controller.pid(0.1, 0.0, 0.0));
            fc.brake.set(Controller.pid(0.1, 0.0, 0.0));
            fc.coast.set(Controller.zero);

            // Required brake coefficient matrices
            fc.linearBrakeCoefficients.set(Matrix.zero(2));
            fc.quadraticBrakeCoefficients.set(Matrix.zero(2));
            fc.headingBrakeCoefficients.set(Vector2D.zero());

            // Required velocity/deceleration values
            fc.maxAchievableForwardVelocity.set(50.0);
            fc.maxAchievableStrafeVelocity.set(35.0);
            fc.naturalForwardDeceleration.set(35.0);
            fc.naturalStrafeDeceleration.set(25.0);

            // Everything else
            fc.maxVelocityConstraint.set(50.0);
            fc.maxAccelerationConstraint.set(40.0);
            fc.maxDecelerationConstraint.set(40.0);
            fc.maxPathSpeed.set(50.0);
            fc.brakeAtEnd.set(true);
            fc.brakeAggression.set(0.8);
            fc.maxBrakingPower.set(0.9);
            fc.maxDecelerationScale.set(1.0);
            fc.minCorrectionDistance.set(0.5);
            fc.headingDeviationTolerance.set(0.05);
            fc.translationalDeviationTolerance.set(1.0);
            fc.holdPointHeadingScaling.set(0.4);
            fc.holdPointTranslationalScaling.set(0.4);
            fc.headingDriveRatio.set(0.3);
            fc.cosineScale.set(true);
        });
    }

    public static Follower create(HardwareMap h) {
        init();
        return new Follower(
                new OTOSLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}