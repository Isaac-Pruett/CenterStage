package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class SlidesManager {
    DcMotorEx left;
    DcMotorEx right;

    PIDFCoefficients PID;
    boolean REVERSE = false;
    public double speed;

    private ElapsedTime time;
    private double last_time = 0;

    private double I = 0;

    double last_error;

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

    public double getHeight(DistanceUnit distanceUnit){
        double avg = (left.getCurrentPosition() + right.getCurrentPosition()) / 2.0;
        // in mm
        double pulley_diameter = 38.2;
        double counts_per_rev = 384.5;
        double mm = (pulley_diameter * Math.PI) / (avg / counts_per_rev);
        return distanceUnit.fromMm(mm);
    }

    public void setSpeed(double speed){
        this.speed = speed;
    }

    public void setPID(PIDFCoefficients PID){
        this.PID = PID;
    }


    public void setHeight(double mm){
        double current = getHeight(DistanceUnit.MM);
        if (current != mm){
            double error = current-mm;
            double P = error * PID.p;
            I += (PID.i * (error * (time.seconds() - last_time)));
            double D = PID.d * (error - last_error) / (time.seconds() - last_time);

            double max_i = 1 - (P+D);
            if (I > max_i) {
                I = max_i;
            }else if (I < -max_i) {
                I = -max_i;
            }

            double output = (P + I + D) * speed;

            last_time = time.seconds();
            last_error = error;

            left.setPower(output);
            right.setPower(output);
        }


    }



}
