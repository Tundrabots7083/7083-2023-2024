package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class PIDController {
    LinearOpMode opMode;
    double Kp;
    double Ki;
    double Kd;
    double lastError = 0;
    double  integralSum = 0;
    boolean angleWrap = false;
    double  integralSumLimit=0.35;
    double lastReference=0;
    double lastTime=0;
    double a = 0.8; // a can be anything from 0 < a < 1
    double previousFilterEstimate = 0;
    double currentFilterEstimate = 0;

    String name="test";

    ElapsedTime timer = new ElapsedTime();

    /**
     * Set PID gains
     * @param Kp proportional gain
     * @param Ki integral gain
     * @param Kd derivative gain
     */
    public PIDController(double Kp, double Ki, double Kd,LinearOpMode opMode, String name) {
        this.opMode = opMode;
        this.name = name;
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

        timer = new ElapsedTime();
    }

    public PIDController(double Kp, double Ki, double Kd, boolean angleWrap,LinearOpMode opMode) {
        this.opMode = opMode;
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.angleWrap = angleWrap;
    }

    /**
     * calculate PID output given the reference and the current system state
     * @param reference where we would like our system to be
     * @param state where our system is
     * @return the signal to send to our motor or other actuator
     */
    public double output(double reference, double state) {
        double error;
        double errorChange;
        double derivative;

        // check if we need to unwrap angle
        if (angleWrap) {
            error = angleWrap(reference - state);
        } else {
            error = reference - state;
        }

        // forward euler integration
        integralSum += error * (timer.seconds() - lastTime);

        // set a limit on our integral sum
        if (integralSum > integralSumLimit) {
            integralSum = integralSumLimit;
        }

        if (integralSum < -integralSumLimit) {
            integralSum = -integralSumLimit;
        }


        //derivative
        // filter out high frequency noise to increase derivative performance
        errorChange = (error - lastError);

        currentFilterEstimate = (a * previousFilterEstimate) + (1-a) * errorChange;

        derivative = (currentFilterEstimate - previousFilterEstimate) / (timer.seconds() - lastTime);

        previousFilterEstimate = currentFilterEstimate;

        double output = (error * Kp) + (integralSum * Ki) + (derivative * Kd);

       // opMode.telemetry.addData("PID", "Name: %s reference: %f  lastReference: %f state: %f integralSum: %f timer.seconds():%f lastTime:%f\n",name, reference, lastReference, state, integralSum,timer.seconds(), lastTime);
      //  opMode.telemetry.update();

       // reset integral sum upon setpoint changes
        if (reference != lastReference) {
            integralSum = 0;
            lastReference = reference;
            timer.reset();
        }

        lastTime=timer.seconds();

        lastError = error;

        return output;
    }


    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }


}