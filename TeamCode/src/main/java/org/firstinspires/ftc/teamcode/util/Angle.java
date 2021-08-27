package org.firstinspires.ftc.teamcode.util;

public class Angle {

    public enum AngleType {
        NEG_180_TO_180,
        ZERO_TO_360,
        NEG_PI_TO_PI,
        ZERO_TO_2PI
    }

    public enum Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }

    private double value;
    private AngleType angleType;

    public Angle() {
        value = 0;
        angleType = AngleType.NEG_180_TO_180;
    }

    public Angle(double value_deg) {
        value = value_deg;
        angleType = AngleType.NEG_180_TO_180;
    }

    public Angle(AngleType angleType) {
        value = 0;
        this.angleType = angleType;
    }

    public Angle(double value_deg, AngleType angleType) {
        value = value_deg;
        this.angleType = angleType;
    }

    public double getValue() {
        return value;
    }

    public AngleType getAngleType() {
        return angleType;
    }

    public double getDifference(Angle angle) {
        return value - angle.getValue();
    }

    public Angle convertTo360() {
        double wrappedVal = value % 360;
        if (wrappedVal < 0) {
            wrappedVal += 360;
        }
        AngleType convertedType = AngleType.ZERO_TO_360;
        return new Angle(wrappedVal, convertedType);
    }

    public Angle convertTo180() {
        double wrappedVal = (value + 180) % 360;
        if (wrappedVal < 0) {
            wrappedVal += 360;
        }
        AngleType convertedType = AngleType.NEG_180_TO_180;
        return new Angle(wrappedVal, convertedType);
    }

    public Direction directionTo (Angle other) {
        Angle otherConverted = other.convertTo360();
        Angle thisConverted = this.convertTo360();

        double rawDiff = Math.abs(otherConverted.getValue() - thisConverted.getValue());
        if (rawDiff > 180) {
            if (otherConverted.getValue() > thisConverted.getValue()) {
                return Direction.CLOCKWISE;
            } else {
                return Direction.COUNTER_CLOCKWISE;
            }
        } else {
            if (otherConverted.getValue() > thisConverted.getValue()) {
                return Direction.COUNTER_CLOCKWISE;
            } else {
                return Direction.CLOCKWISE;
            }
        }
    }
}
