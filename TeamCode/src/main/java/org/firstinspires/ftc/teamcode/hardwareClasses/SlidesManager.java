package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
public class SlidesManager implements subsystem{
    public DcMotorEx left;
    public DcMotorEx right;

    public static double Kp = 10.0;
    public static double Ki = 0.5;
    public static double Kd = 0.0;

    PIDCoefficients pidCoefficients = new PIDCoefficients(Kp, Ki, Kd);
    boolean REVERSE = false;
    public double maxHeight = 700.0;

    double pulley_diameter = 38.2;
    double counts_per_rev = 384.5;

    PIDFController rightPID = new PIDFController(pidCoefficients);
    PIDFController leftPID = new PIDFController(pidCoefficients);

    double target = 0; // in mm

    public SlidesManager(HardwareMap hwmp){
        this.left = hwmp.get(DcMotorEx.class, "left");
        this.right = hwmp.get(DcMotorEx.class, "right");

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (REVERSE){
            left.setDirection(DcMotorSimple.Direction.REVERSE);
            right.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            right.setDirection(DcMotorSimple.Direction.REVERSE);
            left.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }
    public SlidesManager(DcMotorEx l, DcMotorEx r){
        this.left = l;
        this.right = r;

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (REVERSE){
            left.setDirection(DcMotorSimple.Direction.REVERSE);
            right.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            right.setDirection(DcMotorSimple.Direction.REVERSE);
            left.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    public SlidesManager(DcMotorEx l, DcMotorEx r, double max){
        this.left = l;
        this.right = r;

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (REVERSE){
            left.setDirection(DcMotorSimple.Direction.REVERSE);
            right.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            right.setDirection(DcMotorSimple.Direction.REVERSE);
            left.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        this.maxHeight = max;

    }


    private int toTicks(DistanceUnit distanceUnit, double length){
        double mm = distanceUnit.fromMm(length);
        return (int) Math.floor(length / (pulley_diameter * Math.PI / counts_per_rev));
    }

    private double toLength(DistanceUnit distanceUnit, int ticks){
        return distanceUnit.fromMm(ticks * (pulley_diameter * Math.PI / counts_per_rev));
    }

    public void setPidfCoefficients(PIDCoefficients pidCoefficients) {
        this.pidCoefficients = pidCoefficients;
    }

    public void zeroEncoders(){
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setHeightAsync(DistanceUnit distanceUnit, double height){

        double heightPercent = distanceUnit.toMm(height) / distanceUnit.fromMm(maxHeight);

        double leftPercent = toLength(distanceUnit, left.getCurrentPosition()) / distanceUnit.fromMm(maxHeight);

        double rightPercent = toLength(distanceUnit, right.getCurrentPosition()) / distanceUnit.fromMm(maxHeight);

        leftPID.setTargetPosition(heightPercent);
        rightPID.setTargetPosition(heightPercent);

        double leftPower = leftPID.update(leftPercent);

        double rightPower = rightPID.update(rightPercent);

        left.setPower(leftPower);
        right.setPower(rightPower);



    }

    @Override
    public void update() {
        setHeightAsync(DistanceUnit.MM, target);
    }

    public void setTarget(double target) {
        if (target >= 0 && target <= maxHeight){
            this.target = target;
        }
    }

    public void doTelemetry(Telemetry telemetry){
        telemetry.addData("l:", (pulley_diameter * Math.PI / counts_per_rev) * (left.getCurrentPosition()));
        telemetry.addData("r:", (pulley_diameter * Math.PI / counts_per_rev) * (right.getCurrentPosition()));
    }

    public void stop(){
        setRawPower(0);
    }

    private void setRawPower(double power) {
        left.setPower(power);
        right.setPower(power);
    }

    public void zeroSlides(){ //TODO

    }


}
