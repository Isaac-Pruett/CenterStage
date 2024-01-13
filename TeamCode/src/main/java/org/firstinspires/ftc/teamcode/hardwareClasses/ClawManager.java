package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClawManager implements subsystem{
    Servo inner;
    Servo outer;

    double middle = 0.5;
    private final double innerClosed = .67;
    private final double innerOpen = .53;
    private final double outerOpen = innerOpen - .07;
    private final double outerClosed = .67;
    private final double cosmetic = .645;

    public double innerPos;
    public double outerPos;



    public ClawManager(HardwareMap hwmp){
        inner = hwmp.get(Servo.class, "inner");
        outer = hwmp.get(Servo.class, "outer");

    }

    public void close(){
        innerPos = innerClosed;
        outerPos = outerClosed;
    }

    public void open(CLAWS c){
        if (CLAWS.INNER == c){
            outerPos = outerOpen;
            innerPos = innerOpen;
        } else if (c == CLAWS.OUTER) {
            outerPos = outerOpen;
        }
    }

    public void setCosmetic(){
        outerPos = cosmetic;
        innerPos = cosmetic;
    }


    public void setMiddle(){
        innerPos = middle;
        outerPos = middle;
    }

    public enum CLAWS{
        INNER,
        OUTER;
    }

    @Override
    public void update() {
        inner.setPosition(innerPos);
        outer.setPosition(outerPos);
    }

    public void doTelemetry(Telemetry tele){
        tele.addData("outerTARGET", outerPos);
        tele.addData("innerTARGET", innerPos);
        tele.addData("outerSERVO", outer.getPosition());
        tele.addData("innerSERVO", inner.getPosition());
    }

}
