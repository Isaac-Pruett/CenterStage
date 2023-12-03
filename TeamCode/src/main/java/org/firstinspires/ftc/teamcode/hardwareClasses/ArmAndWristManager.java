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

    MODE currentMode;

    public ArmAndWristManager(HardwareMap hardwareMap){
        arm = new ArmManager(hardwareMap);
        wrist = new WristManager(hardwareMap);

    }

    public void setMode(MODE m){
        currentMode = m;
    }

    public void setDepth(DistanceUnit distanceUnit, double depth){ // TODO

        if (currentMode == MODE.PLACING){
            double angleRAD = Math.toRadians(wrist.wristAngle);
            double angleCOS = Math.cos(angleRAD) * clawLength;
            double adj_depth = depth - distanceUnit.fromInches(angleCOS);
            double acosine = Math.toDegrees(Math.acos(adj_depth / armLength));


        }

    }


    @Override
    public void update() {
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
        FLOOR_BACK;
    }

}
