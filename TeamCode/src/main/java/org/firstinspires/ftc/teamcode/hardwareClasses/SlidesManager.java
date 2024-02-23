package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
public class SlidesManager implements subsystem{
    public DcMotorEx left;
    public DcMotorEx right;

    public TouchSensor limitLeft;
    public TouchSensor limitRight;

    boolean REVERSE = false;
    public double maxHeight = 696.0;

    double pulley_diameter = 38.2;
    double counts_per_rev = 384.5;
    double target = 0; // in mm
    double last_target = 0;
    public DistanceUnit dU = DistanceUnit.MM;

    private double power = 1.0;



    public SlidesManager(HardwareMap hwmp){
        this.left = hwmp.get(DcMotorEx.class, "left");
        this.right = hwmp.get(DcMotorEx.class, "right");

        this.limitLeft = hwmp.get(TouchSensor.class, "limitLeft");
        this.limitRight = hwmp.get(TouchSensor.class, "limitRight");

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (REVERSE){
            left.setDirection(DcMotorSimple.Direction.REVERSE);
            right.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            right.setDirection(DcMotorSimple.Direction.REVERSE);
            left.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    private int toTicks(DistanceUnit distanceUnit, double length){
        double mm = distanceUnit.toMm(length);
        return (int) Math.floor(mm / (pulley_diameter * Math.PI / counts_per_rev));
    }
    private double toLength(DistanceUnit distanceUnit, int ticks){
        return distanceUnit.fromMm(ticks * (pulley_diameter * Math.PI / counts_per_rev));
    }

    public boolean isAtTarget(){
        boolean combined = left.isBusy() || right.isBusy();
        return !combined;
    }
    private void zeroEncoders(){
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void setHeightAsync(DistanceUnit distanceUnit, double height){
        if (height != last_target){
            target = height;
            last_target = height;
            left.setTargetPosition(toTicks(distanceUnit, target));
            right.setTargetPosition(toTicks(distanceUnit, target));

            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            setRawPower(1.0);
        }

    }

    public void setPower(double power) {
        this.power = power;
    }

    public double getPower() {
        return power;
    }

    @Override
    public void update() {
        if (target < 0){
            if(!limitsArePressed()){
                left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                setRawPower(-0.8);
            }else{
                stop();
                zeroEncoders();
                target = 0;
            }
        } else{
            setHeightAsync(dU, target);
        }
    }
    public void setTarget(DistanceUnit distanceUnit, double target) {
        this.dU = distanceUnit;
        if (target >= 0 && dU.toMm(target) <= maxHeight){
            this.target = target;
        }
    }
    public void doTelemetry(Telemetry telemetry){

        telemetry.addData("l:", toLength(DistanceUnit.MM, left.getCurrentPosition()));
        telemetry.addData("r:", toLength(DistanceUnit.MM, right.getCurrentPosition()));
        telemetry.addData("leftTicks:", left.getCurrentPosition());
        telemetry.addData("rightTicks:", right.getCurrentPosition());
        telemetry.addData("leftPower:", left.getPower());
        telemetry.addData("rightPower:", right.getPower());
        telemetry.addData("slides target:", target);
        telemetry.addData("l-target:", left.getTargetPosition());
        telemetry.addData("r-target:", right.getTargetPosition());
    }
    public void stop(){
        power = 0;
        setRawPower(0);
    }

    public void zeroSlides(){
        target = -1;
    }

    private void setRawPower(double power) {
        left.setPower(power);
        right.setPower(power);
    }
    private boolean limitsArePressed(){
        return (limitLeft.isPressed() || limitRight.isPressed());
    }

    public void setThreshold(DistanceUnit distanceUnit, double threshold){
        left.setTargetPositionTolerance(toTicks(distanceUnit, threshold));
        right.setTargetPositionTolerance(toTicks(distanceUnit, threshold));
    }

    public double getTarget() {
        return target;
    }
}
