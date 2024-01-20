package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LauncherManager implements subsystem{

    Servo launcher;
    boolean has_fired = false;

    double firing_pos = 1;
    double locked_pos = .5;

    double current_launcher_pos = locked_pos;


    public LauncherManager(HardwareMap hwmp){
        launcher = hwmp.get(Servo.class, "launcher");
    }

    public void lock(){
        current_launcher_pos = locked_pos;
        has_fired = false;
    }


    public void fire(){
        if (!has_fired){
            current_launcher_pos = firing_pos;
            has_fired = true;
        }
    }

    @Override
    public void update() {
        launcher.setPosition(current_launcher_pos);
    }

    @Override
    public void doTelemetry(Telemetry tele) {
        tele.addData("has Fired? = ", has_fired);

    }
}
