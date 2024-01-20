package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmManager implements subsystem {

    Servo armLeft;
    Servo armRight;

    public double armAngle;
    public double angleRange = 355.0;

    public ArmManager(HardwareMap hwmp){
        armLeft = hwmp.get(Servo.class, "armLeft");
        armRight = hwmp.get(Servo.class, "armRight");

        setDirections(Servo.Direction.FORWARD);
    }


    @Override
    public void update() {
        if (armRight.getDirection() == Servo.Direction.FORWARD) {
            armLeft.setPosition(toServoInput(armAngle) + .016);
            armRight.setPosition(toServoInput(armAngle) + .016);
        } else {
            armLeft.setPosition(toServoInput(armAngle) - .016);
            armRight.setPosition(toServoInput(armAngle) - .016);
        }
    }

    public void setArmAngle(double angle){
        if (angle > angleRange/2){
            angle = angleRange/2;
        } else if (angle < -angleRange/2) {
            angle = -angleRange/2;
        }
        armAngle = angle;

    }

    private double toServoInput(double angle){
        return Normalize(angle)/angleRange;
    }
    @Override
    public void doTelemetry(Telemetry tele) {
        tele.addData("Arm Angle = ", armAngle);
        tele.addData("ServoLeft = ", armLeft.getPosition());
        tele.addData("ServoRight = ", armRight.getPosition());
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

    public double convertFromUnitCircle(double angle){
        return ((-1.0 * angle) + 90.0);
    }
    public double convertToUnitCircle(double angle){
        return (angle - 90.0) * -1.0;
    }

    public double getAngleRange() {
        return angleRange;
    }
}
