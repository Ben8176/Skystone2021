package org.firstinspires.ftc.teamcode.util;


public class Vector2d {
    private final double m_x;
    private final double m_y;

    //Vector constants
    final static Vector2d FORWARD = new Vector2d(0, 1),
            BACKWARD = new Vector2d(0, -1),
            LEFT = new Vector2d(-1, 0),
            RIGHT = new Vector2d(1, 0),
            ZERO = new Vector2d(0, 0);

    public Vector2d() {
        this(0,0);
    }

    public Vector2d(double x, double y) {
        m_x = x;
        m_y = y;
    }

    public Vector2d(Vector2d other) {
        m_x = other.getX();
        m_y = other.getY();
    }

    public Vector2d rotateBy(double angle) {
//        angle = Math.toRadians(angle);
        double cosA = Math.cos(angle);
        double sinA = Math.sin(angle);
        double x = m_x * cosA - m_y * sinA;
        double y = m_x * sinA + m_y * cosA;
        return new Vector2d(x,y);
    }

    public Vector2d scale(double scalar) {
        double x = m_x * scalar;
        double y = m_y * scalar;
        return new Vector2d(x,y);
    }

    public Vector2d subtract(Vector2d subtractor) {
        double x = m_x - subtractor.getX();
        double y = m_y - subtractor.getY();
        return new Vector2d(x,y);
    }

    //flips the signs of both components
    public Vector2d reflect () {
        return new Vector2d(-m_x, -m_y);
    }

    //projection of current vector onto v
    public Vector2d projection (Vector2d v) {
        return v.scale(dot(v)/(Math.pow(v.magnitude(), 2))); // u dot v over mag(v)^2 times v
    }

    public Vector2d normalize(double target) {
        if (magnitude() == 0) return ZERO; //avoid dividing by zero
        return scale(target / magnitude());
    }

    //normalizes a group of vectors so that they maintain the same relative magnitudes and ...
    // the vector of largest magnitude now has a magnitude equal to limit
    public static Vector2d[] batchNormalize(double limit, Vector2d... vecs) {
        double maxMag = 0;
        for (Vector2d v : vecs) {
            if (v.magnitude() > maxMag) {
                maxMag = v.magnitude();
            }
        }
        if (limit >= maxMag) {
            return vecs;
        }
        Vector2d[] normed = new Vector2d[vecs.length];
        for (int i = 0; i < vecs.length; i++) {
            normed[i] = vecs[i].scale(limit / maxMag);
        }
        return normed;
    }

    //dot product
    public double dot(Vector2d other) {
        return getX() * other.getX() + getY() * other.getY();
    }

    public double getAngle() {
        return Math.atan2(m_y, m_x);
    }

    public double magnitude() {
        return Math.hypot(m_x, m_y);
    }

    public double getX() {
        return m_x;
    }
    public double getY() {
        return m_y;
    }

}
