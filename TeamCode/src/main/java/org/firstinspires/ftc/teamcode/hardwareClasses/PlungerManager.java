package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PlungerManager implements subsystem {
    public Servo actuator;
    double current = 1.0;
    double retracted = 0.0;
    double halfway = 0.5;
    double extended = 1.0;

    public PlungerManager(HardwareMap hwmp){
        actuator = hwmp.get(Servo.class, "plunger");
    }
    public void retractFully(){
        actuator.setPosition(retracted);
    }
    public void extendFully(){
        actuator.setPosition(extended);
    }
    public void extend(){
        if (current == halfway){
            current = extended;
        } else{
            current = halfway;
        }
    }
    public void retract(){
        if (current == halfway){
            current = retracted;
        } else {
            current = halfway;
        }
    }


    @Override
    public void update() {
        actuator.setPosition(current);
    }

    @Override
    public void doTelemetry(Telemetry tele) {
        tele.addData("plunger current = ", current);
        tele.addData("plunger currentPos = ", actuator.getPosition());
    }
}
