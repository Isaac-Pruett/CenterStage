package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ArmAndWristManager {
    Servo armLeft;
    Servo armRight;
    Servo wrist;

    private double angleRange = 280.0; // in degrees

    private double armLength = 19.0; // in inches

    private double NomalizeTo90Const = (angleRange/2) - 90; // in degrees


    boolean lockedAt0 = false;
    boolean lockedAt30 = false;
    double armAngle = 0;
    double wristAngle = 0;

    double WristOffsetFromZero = 30.0; // estimate for now, in degrees


    public ArmAndWristManager(HardwareMap hwmp){
        armLeft = hwmp.get(Servo.class, "armLeft");
        armRight = hwmp.get(Servo.class, "armRight");
        wrist = hwmp.get(Servo.class, "wrist");

        setDirections(Servo.Direction.FORWARD);
    }

    private double toServoInput(double angle){
        return angle/angleRange;
    }

    public void update(){
        setArmAngle(armAngle);
        setWristAngle(wristAngle);
    }

    public void lockAngleAtZero(){
        lockedAt0 = true;
        lockedAt30 = false;
    }
    public void lockAngleAtThirty(){
        lockedAt0 = false;
        lockedAt30 = true;
    }

    public void setAdjustedWristAngle(double angle){
        if (lockedAt30){
            wristAngle = 150 - armAngle;
        }else if(lockedAt0){
            wristAngle = 180 - armAngle;
        }else{
            wristAngle = angle;
        }
    }

    public void setAdjustedWristAngle(){
        setAdjustedWristAngle(0);
    }


    public void setHeight(DistanceUnit distanceUnit, double height){
        armAngle = Math.toDegrees(Math.asin(distanceUnit.toInches(height)/armLength));
    }

    public void setArmAngle(double degrees){
        armRight.setPosition(toServoInput(degrees));
        armLeft.setPosition(toServoInput(degrees));
    }

    public void setWristAngle(double degrees){
        wrist.setPosition(toServoInput(degrees));
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

    public double getAngleRange() {
        return angleRange;
    }

    public void setAngleRange(double angleRange) {
        this.angleRange = angleRange;
    }
}
