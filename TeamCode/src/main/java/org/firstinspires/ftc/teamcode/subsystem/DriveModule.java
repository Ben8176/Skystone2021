    package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Vector2d;

public class DriveModule {

    DcMotorEx m1, m2;
    double motor1power;
    double motor2power;
    double m0Encoder;
    double m1Encoder;

    private double robot_heading = 0;

    private Vector2d positionVector = new Vector2d(-9, 0);
    private Vector2d targetVec;
    private Vector2d powerVec;

    //Gear ratio constants
    private final double INTERMEDIATE_TO_MOTOR = 2; // 16/8
    private final double MODULE_TO_INTERMEDIATE = 4; // 52/13
    private final double WHEEL_TO_MODULE = 0.25;
    private final double COMMON_GEAR_RATIO = MODULE_TO_INTERMEDIATE * INTERMEDIATE_TO_MOTOR;
    private final double TICKS_PER_REV = 145.6;

    private final double ANGLE_OF_MAX_MODULE_ROTATION_POWER = 110;

    private final double TICKS_PER_MODULE_REV =
            TICKS_PER_REV * INTERMEDIATE_TO_MOTOR * MODULE_TO_INTERMEDIATE;

    private MathUtil myMath = new MathUtil();
    double moduleOrientation;

    private final Vector2d M1_VEC = new Vector2d(1/Math.sqrt(2), 1/Math.sqrt(2));
    private final Vector2d M2_VEC = new Vector2d(-1/Math.sqrt(2), 1/Math.sqrt(2));

    //cant you just make a drivemodule object and pass in different motors
    public DriveModule(DcMotorEx m1, DcMotorEx m2) {
        this.m1 = m1;
        this.m2 = m2;
    }

    public void updateTarget (Vector2d transVec, double rotMag) { //translation vector and rotation magnitude
        //converts robot heading to the angle type used by Vector2d class
        //converts the translation vector from a robot centric to a field centric one
        Vector2d transVecFC = transVec.rotateBy(robot_heading); //was converted robot heading, was clockwise

        //vector needed to rotate robot at the desired magnitude
        //based on positionVector of module (see definition for more info)
        Vector2d rotVec = positionVector.normalize(rotMag).rotateBy(-90); //theoretically this should be rotated 90, not sure sure it doesn't need to be

        //combine desired robot translation and robot rotation to get goal vector for the module
        Vector2d targetVector = transVecFC.add(rotVec);
        targetVec = targetVector;
        //allows modules to reverse power instead of rotating 180 degrees
        //example: useful when going from driving forwards to driving backwards
//        int directionMultiplier = -1; //was positive 1
//        if (reversed) { //reverse direction of translation because module is reversed
//            targetVector = targetVector.reflect();
//            directionMultiplier = 1;
//        }

        //calls method that will apply motor powers necessary to reach target vector in the best way possible, based on current position
        goToTarget(targetVector, 1);
    }


    //sets motor powers for robot to best approach given target vector
    public void goToTarget (Vector2d targetVector, int directionMultiplier) {
        //how much the module needs to translate (and in which direction)
        double moveComponent = targetVector.magnitude() * directionMultiplier;

        //how much the module needs to pivot (change its orientation)
        double pivotComponent;
        if (targetVector.magnitude() != 0) {
            pivotComponent = getPivotComponent(targetVector, moduleOrientation);
        } else {
            //if target vector is zero (joystick is within deadband) don't pivot modules
            pivotComponent = 0;
        }

        //vector in an (invented) coordinate system that represents desired (relative) module translation and module rotation
        Vector2d powerVector = new Vector2d(moveComponent, pivotComponent); //order very important here
        powerVec = powerVector;
        setMotorPowers(powerVector);
    }


    //returns a scalar corresponding to how much power the module needs to apply to rotating
    //this is necessary because of the differential nature of a diff swerve drive
    public double getPivotComponent (Vector2d targetVector, double currentAngle) {
        Angle targetAngle = new Angle(targetVector.getAngle());
        double angleDiff = targetAngle.getValue() - currentAngle; //number from 0 to 180 (always positive)
        Angle currHeadingAngle = new Angle(currentAngle);

        //allows module to rotate to the opposite position of (180 degrees away from) its target
        //if this is the fastest path, we need to indicate that the direction of translation should be reversed
//        if (Math.abs(angleDiff) > 110) { //was 90
//            if (!takingShortestPath) {
//                reversed = !reversed; //reverse translation direction bc module is newly reversed
//            }
//            takingShortestPath = true;
//        } else {
//            takingShortestPath = false;
//        }

        Angle.Direction direction = currHeadingAngle.directionTo(targetAngle);

        //CCW is negative for heading system
        if (angleDiff < 5) {
            //don't rotate module if it's currently within x degrees of its target orientation
            //avoids constant twitching of modules
            return 0;
        } else if (angleDiff > ANGLE_OF_MAX_MODULE_ROTATION_POWER) {
            //rotation power is maxed out if the difference is more than this angle
            if (direction == Angle.Direction.CLOCKWISE) return 1;
            else return -1;
        } else {
            //scale module rotation power based on set constants
            if (direction == Angle.Direction.CLOCKWISE) return angleDiff / ANGLE_OF_MAX_MODULE_ROTATION_POWER;
            else return -1 * angleDiff / ANGLE_OF_MAX_MODULE_ROTATION_POWER;
        }
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

    public void updateModuleOrientation() {
        moduleOrientation = myMath.NEWwrapTo180(-360 * ((m0Encoder + m1Encoder) / 2) / TICKS_PER_MODULE_REV);
    }

    public void update(double m0Encoder, double m1Encoder, double heading) {
        this.m0Encoder = m0Encoder;
        this.m1Encoder = m1Encoder;
        robot_heading = heading;
        updateModuleOrientation();
    }

    public double getModuleOrientation() {
        return moduleOrientation;
    }

    public double getMotor1power() {
        return motor1power;
    }

    public double getMotor2power() {
        return motor2power;
    }

    public Vector2d getTargetVec() {
        return targetVec;
    }

    public Vector2d getPowerVec() {
        return powerVec;
    }
}
