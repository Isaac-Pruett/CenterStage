package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    double p;
    double i;
    double d;

    double integralSum = 0.0;

    double maxIntegralSum = 1;
    double lastError;

    double a = 0.0001; // a can be anything from 0 < a < 1, by default dampening is disabled by setting this to 0
    private double previousFilterEstimate = 0;
    private double lastReference = 0;

    double threshold = 0.10;

    ElapsedTime timer = new ElapsedTime();
    /**
     * construct PID controller
     * @param Kp Proportional coefficient
     * @param Ki Integral coefficient
     * @param Kd Derivative coefficient
     */
    public PIDController(double Kp, double Ki, double Kd) {
        this.p = Kp;
        this.i = Ki;
        this.d = Kd;


    }

    public void setThreshold(double threshold) {
        this.threshold = threshold;
    }

    /**
     * Update the PID controller output.
     * Target and state should be percentages of the maximum position.
     * @param target where we would like to be, also called the reference
     * @param state where we currently are, I.E. motor position
     * @return the command to our motor, I.E. motor power
     */
    public double update(double target, double state) {
        // PID logic and then return the output
        /*
        * Proportional Integral Derivative Controller w/ Low pass filter and anti-windup
        */

        if (Math.abs(target - state) > (threshold/2)){


            // calculate the error
            double error = target - state;
            double errorChange = (error - lastError);


            // filter out high frequency noise to increase derivative performance
            double currentFilterEstimate = (a * previousFilterEstimate) + (1 - a) * errorChange;
            previousFilterEstimate = currentFilterEstimate;


            // rate of change of the error
            double derivative = currentFilterEstimate / timer.seconds();


            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());


            // max out integral sum
            if (integralSum > maxIntegralSum) {
                integralSum = maxIntegralSum;
            }
            if (integralSum < -maxIntegralSum) {
                integralSum = -maxIntegralSum;
            }


            // reset integral sum upon setpoint changes
            if (state != lastReference) {
                integralSum = 0;
            }

            double out = (p * error) + (i * integralSum) + (d * derivative);

            lastError = error;
            lastReference = state;


            // reset the timer for next time
            timer.reset();

            return out;
        } else {
            return 0.0;
        }

    }

}
