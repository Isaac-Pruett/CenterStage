package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WristManager implements subsystem {

    Servo wrist;

    private double wristAngle = 0;
    public final double angleRange = 300.0;

    private static double maxAngle = 140.0;

    private static double minAngle = -160.0;

    public WristManager(HardwareMap hwmp){
        wrist = hwmp.get(Servo.class, "wrist");

        setDirection(Servo.Direction.REVERSE);
    }


    @Override
    public void update() {
        if (wrist.getDirection() == Servo.Direction.FORWARD) {
            wrist.setPosition(toServoInput(wristAngle) + .02);
        } else {
            wrist.setPosition(toServoInput(wristAngle) - .02);
        }

    }

    public void setWristAngle(double angle){
        if (angle > maxAngle){
            angle = maxAngle;
        } else if (angle < minAngle) {
            angle = minAngle;
        }
        wristAngle = angle;

    }

    private double toServoInput(double angle){
        return Normalize(angle)/angleRange;
    }
    @Override
    public void doTelemetry(Telemetry tele) {
        tele.addData("Wrist Angle = ", wristAngle);
        tele.addData("Wrist servo Pos = ", wrist.getPosition());
    }


    public void setDirection(Servo.Direction dir){
        wrist.setDirection(dir);
    }

    private double Normalize(double angle){
        return (angleRange/2) + angle;
    }

    public double getWristAngle() {
        return wristAngle;
    }
}
