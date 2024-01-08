package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ArmAndWristManager implements subsystem {
    ArmManager arm;
    WristManager wrist;

    double armLength = 17.0; // in inches
    double clawLength = 8.0; // in inches TODO: actually tune this

    MODE currentMode = MODE.COSMETIC;

    public ArmAndWristManager(HardwareMap hardwareMap){
        arm = new ArmManager(hardwareMap);
        wrist = new WristManager(hardwareMap);

    }

    public void setMode(MODE m){
        currentMode = m;
    }


    /*
    /**
     * @param distanceUnit unit
     * @param depth can be negative for other side of bot

    public void setClawDepth(DistanceUnit distanceUnit, double depth){ // TODO
        depth = distanceUnit.toInches(depth);
        double height = Math.sqrt(Math.pow(armLength, 2.0) - Math.pow(depth, 2.0));
        if (currentMode == MODE.PLACING){
            double angle_below_horizontal_degrees = -30.0; // for wrist
            double clawCOS = clawLength * Math.cos(Math.toRadians(angle_below_horizontal_degrees));
            double armCOSNeeded = depth-clawCOS;
            double x_axis_locked_arm_angle = Math.toDegrees(Math.acos(armCOSNeeded/armLength));
            arm.armAngle = arm.convertFromUnitCircle(x_axis_locked_arm_angle);
        }else if (currentMode == MODE.FLOOR_BACK) {
            arm.armAngle = arm.convertFromUnitCircle(Math.toDegrees(Math.acos(depth/armLength)));
        } else if (currentMode == MODE.FLOOR_FRONT) {
            arm.armAngle = arm.convertFromUnitCircle(Math.toDegrees(Math.acos(depth/armLength)));
        } else if (currentMode == MODE.COSMETIC){
            arm.armAngle = arm.convertFromUnitCircle(Math.toDegrees(Math.atan2(-height, depth)));
        }

    }

    public void setClawHeight(DistanceUnit distanceUnit, double height){
        height = distanceUnit.toInches(height);
        double depth = Math.sqrt(Math.pow(armLength, 2.0) - Math.pow(height, 2.0));

        if (currentMode == MODE.PLACING){
            double angle_below_horizontal_degrees = -30.0; // for wrist
            double clawSIN = clawLength * Math.sin(Math.toRadians(angle_below_horizontal_degrees));
            double armSINNeeded = height-clawSIN;
            double x_axis_locked_arm_angle = Math.toDegrees(Math.asin(armSINNeeded/armLength));
            arm.armAngle = arm.convertFromUnitCircle(-x_axis_locked_arm_angle);
        }else if (currentMode == MODE.FLOOR_BACK) {
            arm.armAngle = arm.convertFromUnitCircle(Math.toDegrees(Math.atan2(height, -depth)));
        } else if (currentMode == MODE.FLOOR_FRONT) {
            arm.armAngle = arm.convertFromUnitCircle(Math.toDegrees(Math.atan2(height, depth)));
        } else if (currentMode == MODE.COSMETIC){
            arm.armAngle = arm.convertFromUnitCircle(Math.toDegrees(Math.atan2(height, depth)));
        }
    }
    */

    public double getExpectedWristAngle(){
        double theta = arm.convertToUnitCircle(arm.armAngle);
        if (currentMode == MODE.PLACING){
            return theta + 30.0;
        } else if (currentMode == MODE.FLOOR_FRONT){
            //return theta + 90.0;
            if (theta <= 90){
                return theta + 90.0;
            } else{
                return (360 - theta);
            }
        } else if (currentMode == MODE.FLOOR_BACK){
            return theta - 270.0;
        }else if (currentMode == MODE.COSMETIC){
            if (theta <= 90){
                return theta - 90.0;
            } else{
                return (theta - 360.0) - 90.0;
            }
        }else{
            return 0;
        }
    }

    @Override
    public void update() {
        wrist.setWristAngle(getExpectedWristAngle());
        arm.update();
        wrist.update();
    }

    public void doTelemetry(Telemetry tele){
        wrist.doTelemetry(tele);
        arm.doTelemetry(tele);
    }

    enum MODE{
        PLACING,
        FLOOR_FRONT,

        COSMETIC,
        FLOOR_BACK;
    }

}
