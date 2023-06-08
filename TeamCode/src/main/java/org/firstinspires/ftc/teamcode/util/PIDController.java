package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double kP;
    private double kI;
    private double kD;
    private double a = 0.25;
    private double integralSumLimit;

    private ElapsedTime timer;

    private double totalError;
    private double dt;
    private double lastTime;
    private double lastError;
    private double reference;
    private double lastTarget;
    private double lastEstimate;





    public PIDController(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        totalError = 0;
        dt = 0;
        integralSumLimit = 0.8;
        timer = new ElapsedTime();
        lastEstimate = 0;
    }

    public PIDController(double kP, double kI, double kD, double a){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.a = a;
        totalError = 0;
        dt = 0;
        integralSumLimit = 0.8;
        timer = new ElapsedTime();
        lastEstimate = 0;
    }

    public PIDController(double kP, double kI, double kD, double a, double initial){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.a = a;
        totalError = 0;
        dt = 0;
        integralSumLimit = 0.8;
        timer = new ElapsedTime();
        lastEstimate = initial;
    }

    public void setPID(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double calculate(double newMeasurement, double target){
        double currentEstimate = a * lastEstimate + (1 - a) * newMeasurement;
        lastEstimate = currentEstimate;
        if (target != lastTarget){
            timer.reset();
            reference = target;
            lastError = reference - currentEstimate;
            totalError = 0;
        }

        double pTerm = reference - currentEstimate;
        dt = timer.time();
        timer.reset();

        totalError += dt * currentEstimate;

        if (totalError > integralSumLimit){
            totalError = integralSumLimit;
        } else if (totalError < -integralSumLimit){
            totalError = -integralSumLimit;
        }

        double dTerm = (currentEstimate - lastEstimate) / dt;
        lastTarget = reference;

        return kP * pTerm + kI * totalError - kD * dTerm;
    }
}
