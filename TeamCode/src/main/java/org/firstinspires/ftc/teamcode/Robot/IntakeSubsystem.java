package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IntakeSubsystem {
    HardwareMap hwMap;
    DcMotorImplEx intakeMotor;
    ServoImplEx controlServo;
    float intakeAmps;
    public IntakeSubsystem(HardwareMap hardwareMap){
        hardwareMap = hwMap;

        intakeMotor = hwMap.get(DcMotorImplEx.class, "intakeMotor");
        controlServo = hwMap.get(ServoImplEx.class, "controlServo");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void spinUp(){
        intakeMotor.setVelocity(100, AngleUnit.DEGREES);
    }

    public void shutDown(){
        intakeMotor.setPower(0);
    }

    public boolean partIntaked(){
        return intakeMotor.getPower() > intakeAmps;
    }
}
