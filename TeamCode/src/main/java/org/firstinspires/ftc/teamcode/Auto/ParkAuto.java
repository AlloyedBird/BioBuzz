package org.firstinspires.ftc.teamcode.Auto;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class ParkAuto extends LinearOpMode {

    private Follower follower;

    private Pose start;
    private Pose point1;
    private Pose point1Control1;

    private Command autoRoutine() {
        return sequential(
                follow(follower, path1())
        );
    }

    private Path path1() {
        return Paths.curve(start, point1Control1, point1).tangent();
    }

    @Override
    public void runOpMode() {
        Constants.reset();
        Scheduler.reset();
        follower = Constants.create(hardwareMap);

        PoseFactory poseFactory = PoseFactory.degrees();
        start          = poseFactory.of(56, 8, 90);
        point1         = poseFactory.of(7.8123, 101.5451, 166.2103);
        point1Control1 = poseFactory.of(54.8521, 90, 0);

        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path index", follower.pathIndex());
            }

            telemetry.update();
        }
    }
}