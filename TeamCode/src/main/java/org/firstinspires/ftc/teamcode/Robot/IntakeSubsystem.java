package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import java.util.Optional;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingDeque;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Future;
import java.util.concurrent.ThreadPoolExecutor;

public class IntakeSubsystem {
    HardwareMap hwMap;
    DcMotorImplEx motor;
    NormalizedColorSensor colorSensor;
    float redThreshold;
    int MAX_PIECES = 4;
    BlockingQueue<Piece> pieces = new ArrayBlockingQueue<>(MAX_PIECES);
    Thread colorSensorThread = new Thread(() -> colorLoop());


    public IntakeSubsystem(HardwareMap hardwareMap){
        hwMap = hardwareMap;

        motor = hwMap.get(DcMotorImplEx.class, "intakeMotor");
        colorSensor = hwMap.get(NormalizedColorSensor.class, "intakeColorSensor");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
        colorSensorThread.start();
    }

    public void startIntake(){
        motor.setVelocity(1000);
    }

    public void stopIntake(){
        motor.setVelocity(0);
    }


    public BlockingQueue<Piece> loadedPieces(){

        return pieces;
    }
    private void colorLoop(){
        while(true){
            colorSensor.getNormalizedColors();
        }
    }
}
