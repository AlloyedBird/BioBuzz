package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OTOSConfig;
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("frontLeft");
                c.backLeftName.set("backLeft");
                c.frontRightName.set("frontRight");
                c.backRightName.set("backRight");

                //TODO: run tuner and actually test
                c.frontLeftDirection.set(DcMotorImplEx.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorImplEx.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorImplEx.Direction.FORWARD);
                c.backRightDirection.set(DcMotorImplEx.Direction.FORWARD);

                c.manualBrakeMode.set(true);
            }
    );

    public static OTOSConfig localizerConfig = new OTOSConfig(
            c -> {
                c.offset.set(new Pose(0, -8.5, Math.PI / 2));
            }
    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                // Velocity/acceleration constraints
                c.maxVelocityConstraint.set(50.0);
                c.maxAccelerationConstraint.set(40.0);
                c.maxDecelerationConstraint.set(40.0);
                c.maxPathSpeed.set(50.0);

                // Natural deceleration (tune with autotune tomorrow)
                c.naturalForwardDeceleration.set(35.0);
                c.naturalStrafeDeceleration.set(25.0);

                // Achievable velocities (tune with autotune tomorrow)
                c.maxAchievableForwardVelocity.set(50.0);
                c.maxAchievableStrafeVelocity.set(35.0);

                // Braking behavior
                c.brakeAtEnd.set(true);
                c.brakeAggression.set(0.8);
                c.maxBrakingPower.set(0.9);
                c.maxDecelerationScale.set(1.0);

                // Correction tolerances
                c.minCorrectionDistance.set(0.5);
                c.headingDeviationTolerance.set(0.05);
                c.translationalDeviationTolerance.set(1.0);

                // Hold point scaling (how hard it tries to hold end position)
                c.holdPointHeadingScaling.set(0.4);
                c.holdPointTranslationalScaling.set(0.4);

                // Drive ratio (how much it prioritizes heading vs translation)
                c.headingDriveRatio.set(0.3);

                // Cosine scaling helps with mecanum path following
                c.cosineScale.set(true);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new OTOSLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}