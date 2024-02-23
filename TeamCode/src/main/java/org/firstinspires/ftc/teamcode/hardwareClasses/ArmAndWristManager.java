package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ArmAndWristManager implements subsystem {
    public ArmManager arm;
    public WristManager wrist;

    // both of these values are from mounting points,
    // as if each were a straight,
    // perfectly rigid line.
    // (definition of the magnitude of a slope)

    double armLength = 19.0; // in inches
    double clawLength = 4.5; // in inches

    private MODE currentMode = MODE.COSMETIC;

    public ArmAndWristManager(HardwareMap hardwareMap){
        arm = new ArmManager(hardwareMap);
        wrist = new WristManager(hardwareMap);

    }

    public void setMode(MODE m){
        currentMode = m;
    }

    public MODE getMode(){
        return currentMode;
    }


    /*  TODO: this has been commented out bc of simplicity,
        TODO: it should probably be implemented for auto and driving,
        TODO: as it is the ideal control scheme.

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

    /**
     * @return expected wrist angle (in the angle input that the wrist understands) given a MODE and armAngle.
     * Mind the servo direction.
     */
    public double getExpectedWristAngle(){
        double theta = arm.convertToUnitCircle(arm.armAngle);
        if (currentMode == MODE.PLACING){
            //if we are placing on the board,
            return theta + 30.0;
        } else if (currentMode == MODE.FLOOR_BACK){
            // this calculation is the correct one.
            // see arm.convertToUnitCircle().
            // the reason why is bc
            // arm.convertToUnitCircle(135DEG).
            // = (135DEG-90DEG) * -1 == -45DEG,
            // which makes perfect sense.
            return theta + 90.0;
            // this is slightly complicated. also WRONG.
            // as the unit circle input doesnt account for negative angles,
            // so we have to make a negative angle if it makes no sense.
            // basically, if the unit circle (arm angle) drops below 0DEG,
            // we know that it is > 90DEG, because if we place on the front side,
            // it doesnt make sense to have the arm pointing backward.
            // in fact, it could be even 45DEG, but 90DEG makes more sense atm.
            /*
            if (theta <= 90){
                return theta + 90.0;
            } else{
                return (360 - theta);
            }
            */
        } else if (currentMode == MODE.FLOOR_FRONT){
            return theta - 270.0;
        } else if (currentMode == MODE.COSMETIC){
            return theta - 90.0;
            //see case for MODE.FLOOR_FRONT.
            /*
            if (theta <= 90){
                return theta - 90.0;
            } else{
                return (theta - 360.0) - 90.0;
            }
            */
        } else if (currentMode == MODE.POKE) {
            return theta - 285.0;

        } else{
            return wrist.getWristAngle();
        }
    }

    @Override
    public void update() {
        wrist.setWristAngle(getExpectedWristAngle());
        arm.update();
        wrist.update();
    }

    public void doTelemetry(Telemetry tele){
        tele.addData("Current mode = ", currentMode.name());
        tele.addData("arm unit circle angle = ", arm.convertToUnitCircle(arm.armAngle));
        wrist.doTelemetry(tele);
        arm.doTelemetry(tele);
    }

    public enum MODE{
        PLACING,
        FLOOR_FRONT,

        COSMETIC,
        FLOOR_BACK,
        POKE,
        MANUAL;

    }

}
