package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LauncherManager implements subsystem{

    Servo trigger;
    boolean trigger_status = false;

    double firing_pos = 1;
    double locked_pos = .5;

    double current_trigger_pos;


    public LauncherManager(HardwareMap hwmp){
        trigger = hwmp.get(Servo.class, "trigger");
    }

    public void lock(){
        current_trigger_pos = locked_pos;
        trigger_status = false;
    }


    public void fire(){
        if (!trigger_status){
            current_trigger_pos = firing_pos;
            trigger_status = true;
        }
    }

    @Override
    public void update() {
        trigger.setPosition(current_trigger_pos);
    }

    @Override
    public void doTelemetry(Telemetry tele) {
        tele.addData("has Fired? = ", trigger_status);

    }
}
