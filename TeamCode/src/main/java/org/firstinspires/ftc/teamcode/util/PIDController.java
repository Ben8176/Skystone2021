package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    private double setPoint;
    private double kP;
    private double kI;
    private double kD;
    private double lastError;
    private double lastTime;
    ElapsedTime time = new ElapsedTime();

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        lastError = 0;
        lastTime = 0;
    }

    public PIDController(double setPoint, double kP, double kI, double kD) {
        this.setPoint = setPoint;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        lastError = 0;
        lastTime = 0;
    }

    public PIDController() {
        lastError = 0;
        lastTime = 0;
    }

    public double calculate(double input) {
        double error = setPoint - input;
        double currTime = time.milliseconds();
        double period = currTime - lastTime;
        double errorSum = error * period;
        double errorSlope = (error - lastError) / period;
        lastError = error;
        lastTime = currTime;

        return kP * error + kI * errorSum + kD * errorSlope;
    }

    public double calculate(double input, double _setPoint) {
        double error = _setPoint - input;
        double currTime = time.milliseconds();
        double period = currTime - lastTime;
        double errorSum = error * period;
        double errorSlope = (error - lastError) / period;
        lastError = error;
        lastTime = currTime;

        return kP * error + kI * errorSum + kD * errorSlope;
    }

    public double calculateWithError(double error, double kP, double kI, double kD) {
        double currTime = time.milliseconds();
        double period = currTime - lastTime;
        double errorSum = error * period;
        double errorSlope = (error - lastError) / period;
        lastError = error;
        lastTime = currTime;

        return kP * error + kI * errorSum + kD * errorSlope;
    }
}
