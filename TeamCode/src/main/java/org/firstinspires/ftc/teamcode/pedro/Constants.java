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

    static MecanumConfig drivetrainConfig;
    static OTOSConfig localizerConfig;
    private static ForesightConfig foresightConfig;

    public static void reset() {
        drivetrainConfig = null;
        localizerConfig = null;
        foresightConfig = null;
    }

    public static void init() {
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