package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Vector2d;

public class LeftModule {

    DcMotorEx m1, m2;
    double motor1power;
    double motor2power;

    private final Vector2d M1_VEC = new Vector2d(1/Math.sqrt(2), 1/Math.sqrt(2));
    private final Vector2d M2_VEC = new Vector2d(-1/Math.sqrt(2), 1/Math.sqrt(2));

    public LeftModule(DcMotorEx m1, DcMotorEx m2) {
        this.m1 = m1;
        this.m2 = m2;
    }

    public void setMotorPowers(Vector2d target) {
        Vector2d motor1Unscaled = target.projection(M1_VEC);
        Vector2d motor2Unscaled = target.projection(M2_VEC);

        //makes sure no vector magnitudes exceed the maximum motor power
        Vector2d[] motorPowersScaled = Vector2d.batchNormalize(0.7, motor1Unscaled, motor2Unscaled);
        motor1power = motorPowersScaled[0].magnitude();
        motor2power = motorPowersScaled[1].magnitude();

        //this is to add sign to magnitude, which returns an absolute value
        if (motorPowersScaled[0].getAngle() != M1_VEC.getAngle()) {
            motor1power *= -1;
        }
        if (motorPowersScaled[1].getAngle() != M2_VEC.getAngle()) {
            motor2power *= -1;
        }

        m1.setPower(motor1power);
        m2.setPower(motor2power);
    }

    public double getMotor1power() {
        return motor1power;
    }

    public double getMotor2power() {
        return motor2power;
    }
}
