package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FlowerSubsystem {
    HardwareMap hwMap;
    DcMotorImplEx flowerLaunchMotor;
    Servo initServo;
    public FlowerSubsystem(HardwareMap hardwareMap){
        hwMap = hardwareMap;

        flowerLaunchMotor = hwMap.get(DcMotorImplEx.class, "flowerMotor");
        flowerLaunchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        initServo = hwMap.get(Servo.class, "flowerInitServo");
    }

    public void initMechanism(){
        initServo.setPosition(1);
    }

    public void startFill(){
        flowerLaunchMotor.setVelocity(500);
    }
    public void reverseMotor(){flowerLaunchMotor.setPower(-1);}

    public void flowerDisengage(){
        initServo.setPosition(0);
    }

    public void fullStop(){
        flowerDisengage();
        flowerLaunchMotor.setVelocity(0);
    }
}
