package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawManager {
    Servo inner;
    Servo outer;

    double middle = 0.5;
    public double differenceFromMiddle = 0.1;


    public ClawManager(HardwareMap hwmp){
        inner = hwmp.get(Servo.class, "inner");
        outer = hwmp.get(Servo.class, "outer");

    }

    public void close(CLAWS c){
        inner.setPosition(middle-differenceFromMiddle);
        if (c == CLAWS.OUTER){
            outer.setPosition(middle-differenceFromMiddle);
        }
    }

    public void open(CLAWS c){
        if (c == CLAWS.INNER && outer.getPosition() != middle-differenceFromMiddle){
            outer.setPosition(middle+differenceFromMiddle);
            inner.setPosition(middle+differenceFromMiddle);
        } else if (c == CLAWS.OUTER){
            outer.setPosition(middle+differenceFromMiddle);
        }
    }

    public enum CLAWS{
        INNER,
        OUTER;
    }

}
