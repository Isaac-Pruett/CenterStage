package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmManager implements subsystem {

    Servo armLeft;
    Servo armRight;

    public double armAngle;
    private double angleRange = 300.0;

    public ArmManager(HardwareMap hwmp){
        armLeft = hwmp.get(Servo.class, "armLeft");
        armRight = hwmp.get(Servo.class, "armRight");

        setDirections(Servo.Direction.FORWARD);
    }


    @Override
    public void update() {
        armLeft.setPosition(toServoInput(armAngle));
        armRight.setPosition(toServoInput(armAngle));
    }

    public void setArmAngle(double angle){
        if (armAngle >= -angleRange/2 && armAngle <= angleRange/2){
            armAngle = angle;
        }
    }

    private double toServoInput(double angle){
        return Normalize(angle)/angleRange;
    }
    @Override
    public void doTelemetry(Telemetry tele) {
        tele.addData("Arm Angle = ", armAngle);
    }


    public void setDirections(Servo.Direction dir){
        if (dir == Servo.Direction.FORWARD){
            armLeft.setDirection(Servo.Direction.REVERSE);
            armRight.setDirection(Servo.Direction.FORWARD);
        } else{
            armLeft.setDirection(Servo.Direction.FORWARD);
            armRight.setDirection(Servo.Direction.REVERSE);
        }
    }

    private double Normalize(double angle){
        return (angleRange/2) + angle;
    }

    public double getAngleRange() {
        return angleRange;
    }
}
