package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class SlidesManager {
    public DcMotorEx left;
    public DcMotorEx right;

    public static double Kp = 1.0;
    public static double Ki = 0.0;
    public static double Kd = 0.0;

    PIDFCoefficients PID = new PIDFCoefficients(Kp, Ki, Kd, 0.0);
    boolean REVERSE = false;
    public double speed;

    public double maxHeight = 700.0;

    double pulley_diameter = 38.2;
    double counts_per_rev = 384.5;
    private double threshold = 0.01;

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

    public SlidesManager(DcMotorEx l, DcMotorEx r, double max, boolean reverse){
        REVERSE = reverse;
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
        return distanceUnit.fromMm(Math.floor(ticks * (pulley_diameter * Math.PI / counts_per_rev)));
    }

    public void setPID(PIDFCoefficients PID) {
        this.PID = PID;
    }
    public void setThreshold(double t){
        this.threshold = t;
    }
    public void setThreshold(DistanceUnit distanceUnit, double t){
        this.setThreshold(distanceUnit.toMm(t) / maxHeight);
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

        PIDController leftPID = new PIDController(PID.p, PID.i, PID.d);
        leftPID.threshold = threshold;


        double leftPower = leftPID.update(heightPercent, leftPercent);

        PIDController rightPID = new PIDController(PID.p, PID.i, PID.d);
        rightPID.threshold = threshold;

        double rightPower = rightPID.update(heightPercent, rightPercent);

        left.setPower(leftPower);
        right.setPower(rightPower);



    }

    public void doTelemetry(Telemetry telemetry){
        telemetry.addData("l:", (pulley_diameter * Math.PI / counts_per_rev) * (left.getCurrentPosition()));
        telemetry.addData("r:", (pulley_diameter * Math.PI / counts_per_rev) * (right.getCurrentPosition()));
    }




}
