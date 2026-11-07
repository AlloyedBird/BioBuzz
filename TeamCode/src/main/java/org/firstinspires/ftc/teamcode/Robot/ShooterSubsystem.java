package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Annotations.DEV;

@DEV
public class ShooterSubsystem {
    HardwareMap hwMap;
    DcMotorImplEx shooterMotor;
    ServoImplEx gateServo;
    public double MOTOR_VELOCITY = 10;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        hwMap = hardwareMap;

        shooterMotor = hwMap.get(DcMotorImplEx.class, "ShooterMotor");
        //shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        gateServo = hwMap.get(ServoImplEx.class, "gateServo");
    }

    public void spinUp(){
        shooterMotor.setVelocity(MOTOR_VELOCITY, AngleUnit.DEGREES);
    }

    public void spinDown(){
        shooterMotor.setPower(0);
    }
    public void openGate(){
        gateServo.setPosition(0.75);
    }
    public void closeGate(){
        gateServo.setPosition(0);
    }
}
