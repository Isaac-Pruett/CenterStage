package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WristManager implements subsystem {

    Servo wrist;

    public double wristAngle;
    private final double angleRange = 300.0;

    public static double maxRange = 300.0 - 60.0;

    public WristManager(HardwareMap hwmp){
        wrist = hwmp.get(Servo.class, "wrist");

        setDirection(Servo.Direction.FORWARD);
    }


    @Override
    public void update() {
        wrist.setPosition(toServoInput(wristAngle));

    }

    public void setWristAngle(double angle){
        if (wristAngle >= -maxRange/2 && wristAngle <= maxRange/2){
            wristAngle = angle;
        }
    }

    private double toServoInput(double angle){
        return Normalize(angle)/angleRange;
    }
    @Override
    public void doTelemetry(Telemetry tele) {
        tele.addData("Wrist Angle = ", wristAngle);
    }


    public void setDirection(Servo.Direction dir){
        wrist.setDirection(dir);
    }

    private double Normalize(double angle){
        return (angleRange/2) + angle;
    }
}
