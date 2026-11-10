package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Annotations.PROD;

@TeleOp(name = "Rocco zero damn servo", group = "rocco util")
@PROD

public class ZeroRoccoServo extends OpMode {
    ServoImplEx servo;
    DcMotorImplEx motor;
    @Override
    public void init(){
        servo = hardwareMap.get(ServoImplEx.class, "servo");
        motor = hardwareMap.get(DcMotorImplEx.class, "motor");
    }

    @Override
    public void loop() {
        servo.setPosition(0);
        motor.setPower(1);
    }
}
