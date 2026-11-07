package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OTOSConfig;
import com.pedropathing.revhub.localizers.OTOSLocalizer;
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

            }
    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {

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