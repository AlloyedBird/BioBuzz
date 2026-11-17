package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "motor test type shi", group = "testing")
public class MotorTest extends OpMode  {
    private DcMotorImplEx frontLeft = null;
    private DcMotorImplEx frontRight = null;
    private DcMotorImplEx backLeft = null;
    private DcMotorImplEx backRight = null;
    @Override
    public void init(){
        frontLeft = hardwareMap.get(DcMotorImplEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorImplEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorImplEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorImplEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop(){
        if (gamepad1.a){
            frontLeft.setPower(0.1);
        }
        else if (gamepad1.b) {
            frontRight.setPower(0.1);
        }
        else if (gamepad1.x){
            backLeft.setPower(0.1);
        }
        else if (gamepad1.y) {
            backRight.setPower(0.1);
        }
        else{
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }
}
