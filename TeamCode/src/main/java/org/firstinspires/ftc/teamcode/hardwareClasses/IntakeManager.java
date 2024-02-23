package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeManager implements subsystem{

    Servo leftPaddle;
    Servo rightPaddle;
    boolean REVERSED = true;
    double open = 1.0-.03;
    double closed = .39;
    public double currentpos = open;

    public IntakeManager(HardwareMap hwmp){
        leftPaddle = hwmp.get(Servo.class, "leftPaddle");
        rightPaddle = hwmp.get(Servo.class, "rightPaddle");
        if (REVERSED){
            leftPaddle.setDirection(Servo.Direction.FORWARD);
            rightPaddle.setDirection(Servo.Direction.REVERSE);
        } else {
            leftPaddle.setDirection(Servo.Direction.REVERSE);
            rightPaddle.setDirection(Servo.Direction.FORWARD);
        }
    }
    @Override
    public void update() {
        leftPaddle.setPosition(currentpos);
        rightPaddle.setPosition(currentpos);
    }

    public void close(){
        currentpos = closed;
    }
    public void open(){
        currentpos = open;
    }

    @Override
    public void doTelemetry(Telemetry tele) {
        tele.addData("current paddle pos = ", currentpos);

    }
}
