package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class MathUtil {

    private final double EPSILON = 1e-6;
    public final static double TAU = 2 * Math.PI;
    //tunable curve steepness constant
    private final double kSigma = 1.05;
    public MathUtil() {
    }

    /*
    Normalize an array of double inputs to a max of 1
     */
    public double[] normalize(double[] inputs) {
        double[] outputs = new double[inputs.length];
        double max = 0;
        for (double d : inputs) {
            if (Math.abs(d) > max) {
                max = Math.abs(d);
            }
        }

        for (int i = 0; i < inputs.length; i++) {
            if (max != 0) {
                outputs[i] = inputs[i] / max;
            }
        }
        return outputs;
    }

    /*
    Generate a smooth motion profile to follow using the initial error
    y-axis is motor power modifier
    input is current error from target, output is y value at that input value
     */
    public double powerMotionProfile(double currentError, double initialError) {
        return sigma(generateRoots(currentError, initialError));
    }

    /*
    Generate an absolute value function shaped like "\/" which has two x intercepts
    X intercepts are 0 and initial error value
     */
    public double generateRoots(double currentError, double initialError) {
        return -Math.abs(currentError - (initialError/2)) + initialError/2;
    }

    /*
    Smooth motion profile curve using the absolute value function as its input
     */
    public double sigma(double errorInput) {
        return (1.8 * Math.pow(kSigma, errorInput)) / (Math.pow(kSigma, errorInput) + 1) - 0.8;
    }

    public boolean epsilonEquals(double a, double b) {
        return Math.abs(a - b) < EPSILON;
    }

    public double wrapToTau(double angle) {
        return angle % TAU;
    }

    public double wrapTo360(double angle) {
        return angle >= 0 ? angle % 360 : angle + 360;
    }

    /*Doesn't work, deprecated
    public double wrapTo180(double angle) {
        return ((angle + 180) % 360) - 180;
    }
    */

    /*
    fix weird angle wrapping in this new method
     */
    public double NEWwrapTo180(double angle) {
        double result;
        double alpha = angle % 360;

        if (alpha > 180) {
            result = -1 * (360 - alpha);
        }
        else if (alpha < -180) {
            result = 360 + alpha;
        }
        else {
            result = alpha;
        }
        return result;
    }

    public double atan360(double y, double x) {
        return wrapTo360(Math.toDegrees(Math.atan2(y, x)));
    }

    public double sinDeg(double angle) {
        return Math.sin(Math.toRadians(angle));
    }

    public double cosDeg(double angle) {
        return Math.cos(Math.toRadians(angle));
    }

    public double magnitude(double x, double y) {
        return Math.sqrt(x*x + y*y);
    }
}