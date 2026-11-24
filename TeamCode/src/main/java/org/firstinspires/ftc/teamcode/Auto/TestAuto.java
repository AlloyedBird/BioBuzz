package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Drive;
import org.firstinspires.ftc.teamcode.Robot.Drivetrain;

@Autonomous(name = "Test auto", group = "test")
public class TestAuto extends OpMode {
Drivetrain drivetrain;
    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        drivetrain.resetOdometry(0,0,0);
    }

    @Override
    public void loop() {
        drivetrain.moveToPose(0, 24, 90, 1);
    }
}
