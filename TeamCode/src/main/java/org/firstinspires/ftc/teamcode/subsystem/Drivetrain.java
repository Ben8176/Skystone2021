package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.localization.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.TimerUtil;
import org.firstinspires.ftc.teamcode.util.Vector2d;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import static org.firstinspires.ftc.teamcode.util.MathUtil.TAU;

@Config
public class Drivetrain {


    private static double tempPModule = -0.0006;
    private static double tempIModule = 0;
    private static double tempDModule = 0;
    public static double kpRobot = 0.5;
    public static double kiRobot = 0;
    public static double kdRobot = 0;

    MathUtil myMath = new MathUtil();
    TimerUtil myTimer = new TimerUtil();
    PIDController modulePID = new PIDController(tempPModule, tempIModule, tempDModule);
    PIDController robotPID = new PIDController();

    ExpansionHubMotor m0;
    ExpansionHubMotor m1;
    ExpansionHubMotor m2;
    ExpansionHubMotor m3;
    RevBulkData cdata, edata;
    ThreeWheelLocalizer localizer;

    boolean foo = false;
    private boolean autoAim = false;


    //misc. constants
    private final double TRACK_WIDTH = 301; //wheelbasediameter in mm
    private final double WHEEL_DIAMETER = 64; //mm
    private final double MAX_WHEEL_RPM = 550;
    private final double MAX_ROBOT_RPM = 60;
    private final double MAX_MODULE_RPM = 50;
    private final double MAX_MOTOR_RPM = 1150;
    private final double TICKS_PER_REV = 145.6;

    private final double SMALL_ANGLE_THRESH = 30;

    //Gear ratio constants
    private final double INTERMEDIATE_TO_MOTOR = 2; // 16/8
    private final double MODULE_TO_INTERMEDIATE = 4; // 52/13
    private final double WHEEL_TO_MODULE = 0.25;
    private final double COMMON_GEAR_RATIO = MODULE_TO_INTERMEDIATE * INTERMEDIATE_TO_MOTOR;

    private final double TICKS_PER_MODULE_REV =
            TICKS_PER_REV * INTERMEDIATE_TO_MOTOR * MODULE_TO_INTERMEDIATE;

    // PID constants
    //P constant for robot turning
    private double kP_Robot = 0.7;
    //D constant for robot turning
    private double kD_Robot = 60;

    private final double SA_kP_Robot = 20;
    private final double SA_kD_Robot = 30;

    /*
    private double kP_Robot = 1.2;
    //D constant for robot turning
    private double kD_Robot = 340;
     */
    //P constant for module turning
    private final double kP_Module = 0.12;
    //derivative constant for module turning
    private final double kD_Module = 0.7;

    double leftSign = 1;
    double rightSign = 1;
    int leftInvert = 1, rightInvert = 1;
    double leftModulePhi, rightModulePhi;
    double prevLeftAlpha = 0;
    double prevRightAlpha = 0;
    double prevRotAlpha = 0;
    double prevTime = 0;
    double leftModuleTheta = 0;
    double transLeftTheta = 0;
    double robotD = 0;
    double robotP = 0;
    private double leftWheelRPM = 0;
    private double rightWheelRPM = 0;

    private double rawLeftModulePhi = 0;

    public static double m0Encoder;
    public static double m1Encoder;
    public static double m2Encoder;
    public static double m3Encoder;

    public static double m0Offset = 0;
    private double m1Offset = 0;
    private double m2Offset = 0;
    private double m3Offset = 0;

    public static double lastAngle = 0;
    private double lastTransAngle = 0;

    Telemetry driveTelem;

    public Drivetrain(Telemetry telemetry, ExpansionHubMotor m0, ExpansionHubMotor m1, ExpansionHubMotor m2, ExpansionHubMotor m3, ThreeWheelLocalizer localizer) {
        this.m0 = m0;
        this.m1 = m1;
        this.m2 = m2;
        this.m3 = m3;
        this.localizer = localizer;
        driveTelem = telemetry;
    }

    /**
     * @param rotAngle   desired angle to rotate to, in degrees
     * @param transAngle desired angle to translate at, in degrees
     * @param transPow   magnitude of power to translate at
     * @param givePower  enables drive motors, used for debugging
     */
    public void updateSwerve(double rotAngle, double rotPower, double transAngle, double transPow, boolean givePower) {

        double rotSign;
        double robotRPM;
        double rotLeftTheta;
        double rotRightTheta;

        double currentTime = myTimer.getMillis();

        //set transpow between 0 and 1
        double transPower = Range.clip(transPow, 0.0, 1.0);

//        if (autoAim) {
//            rotAngle = angleDiff();
//            rotPower = 1;
//        };
        if (rotPower > 0.1){
            lastAngle = rotAngle;
        }
        else {
            rotAngle = lastAngle;
        }

        if (transPow > 0.1) {
            lastTransAngle = transAngle;
        }
        else {
            transAngle = lastTransAngle;
        }

        if (transPow < 0.1 && rotPower < 0.1) {
            givePower = false;
        }

        //current robot heading
        double rotPhi = Math.toDegrees(globalPosition().getTheta());
        // difference in desired robot heading and current robot heading
        double rotAlpha = myMath.NEWwrapTo180(rotAngle - rotPhi);

        if (rotAlpha < 0) {
            rotSign = -1;
        } else if (rotAlpha != 0) {
            rotSign = 1;
        }// Needed to avoid divide by 0 error
        else {
            rotSign = 0;
        }

        //get updates for encoder numbers
        //calculate left phi and right phi modules
//        update();

        if (Math.abs(rotAlpha) > SMALL_ANGLE_THRESH) {
            kD_Robot = SA_kD_Robot;
            kP_Robot = SA_kP_Robot;
        }
        else {
            kP_Robot = 0.7;
            kD_Robot = 60;
        }

//        //Derivative term for robot rotation
//        robotD = kD_Robot * (rotAlpha - prevRotAlpha) / (currentTime - prevTime);
////        robotD = 0;
//        //Proportional term for robot rotation
//        robotP = kP_Robot * rotAlpha;
//
//        //set the robot RPM to either MAX RPM or some P loop with angle input
//        robotRPM = Math.min(Math.abs(robotP + robotD), MAX_ROBOT_RPM);
        robotRPM = Math.min(Math.abs(robotPID.calculateWithError(rotAlpha, kpRobot, kiRobot, kdRobot)), MAX_ROBOT_RPM);

        //get module direction in order to turn robot

        if (rotSign != 0) {
            rotLeftTheta = 90 + (90 * rotSign);
            rotRightTheta = 90 - (90 * rotSign);
        } else {
            rotLeftTheta = leftModulePhi;
            rotRightTheta = rightModulePhi;
        }

        double rotWheelRPM = robotRPM * (TRACK_WIDTH / WHEEL_DIAMETER);


        transLeftTheta = myMath.NEWwrapTo180(transAngle - rotPhi);
        double transRightTheta = myMath.NEWwrapTo180(transAngle - rotPhi);
        //think of something better to get full powers
        double transWheelRPM = transPower * (MAX_WHEEL_RPM - Math.abs(rotWheelRPM));

        //x y component of left module
        double leftX = rotWheelRPM * myMath.cosDeg(rotLeftTheta) + transWheelRPM * myMath.cosDeg(transLeftTheta);
        double leftY = rotWheelRPM * myMath.sinDeg(rotLeftTheta) + transWheelRPM * myMath.sinDeg(transLeftTheta);
        //x y component of right module
        double rightX = rotWheelRPM * myMath.cosDeg(rotRightTheta) + transWheelRPM * myMath.cosDeg(transRightTheta);
        double rightY = rotWheelRPM * myMath.sinDeg(rotRightTheta) + transWheelRPM * myMath.sinDeg(transRightTheta);

        //get final module headings and RPM from components

        leftModuleTheta = Math.toDegrees(Math.atan2(leftY, leftX));
        leftWheelRPM = myMath.magnitude(leftX, leftY);
        double rightModuleTheta = Math.toDegrees(Math.atan2(rightY, rightX));
        rightWheelRPM = myMath.magnitude(rightX, rightY);

        double leftAlpha = myMath.NEWwrapTo180(leftModuleTheta - leftModulePhi);
        double rightAlpha = myMath.NEWwrapTo180(rightModuleTheta - rightModulePhi);

        //check which quadrant leftalpha is in
        if (leftAlpha > 90) { //quadrant 2
            leftSign = -1;
            leftInvert = -1;
            leftAlpha = myMath.NEWwrapTo180(leftAlpha + 180);
        } else if (leftAlpha < -90) { //quadrant 3
            leftSign = 1;
            leftInvert = -1;
            leftAlpha = myMath.NEWwrapTo180(leftAlpha + 180);
        } else if (leftAlpha > 0) { //quadrant 1
            leftSign = 1;
            leftInvert = 1;
            leftAlpha = myMath.NEWwrapTo180(leftAlpha);
        } else if (leftAlpha != 0) { //quadrant 4
            leftSign = -1;
            leftInvert = 1;
            leftAlpha = myMath.NEWwrapTo180(leftAlpha);
        } else {
            leftSign = 0;
        }

        //check which quadrant rightAlpha is in
        if (rightAlpha > 90) { //quadrant 2
            rightSign = -1;
            rightInvert = -1;
            rightAlpha = myMath.NEWwrapTo180(rightAlpha + 180);
        } else if (rightAlpha < -90) { //quadrant 3
            rightSign = 1;
            rightInvert = -1;
            rightAlpha = myMath.NEWwrapTo180(rightAlpha + 180);
        } else if (rightAlpha > 0) { //quadrant 1
            rightSign = 1;
            rightInvert = 1;
            rightAlpha = myMath.NEWwrapTo180(rightAlpha);
        } else if (rightAlpha != 0) { //quadrant 4
            rightSign = -1;
            rightInvert = 1;
            rightAlpha = myMath.NEWwrapTo180(rightAlpha);
        } else {
            rightAlpha = 0;
        }

        /*
        Derivative control for module turning pid
         */
        double leftModuleD = kD_Module * (leftAlpha - prevLeftAlpha) / (currentTime - prevTime);
        double rightModuleD = kD_Module * (rightAlpha - prevRightAlpha) / (currentTime - prevTime);

        rightSign = Math.copySign(1, -rightAlpha);
        leftSign = Math.copySign(1, -leftAlpha);

        //get module RPMs based on angle and max module rpm
//        double leftRPM  = Math.min(kModule * Math.abs(myMath.NEWwrapTo180()(leftAlpha)),  MAX_MODULE_RPM);
        double leftRPM = leftModuleD + kP_Module * Math.abs(myMath.NEWwrapTo180(leftAlpha));
//        double rightRPM = Math.min(kModule * Math.abs(myMath.NEWwrapTo180()(rightAlpha)), MAX_MODULE_RPM);
        double rightRPM = rightModuleD + kP_Module * Math.abs(myMath.NEWwrapTo180(rightAlpha));

        //normalize RPMs to be below MAX_MOTOR_RPM
        //0.05
        double wheelModifier = 0.05;
//                1 - (Math.max(leftRPM, rightRPM) * MODULE_TO_INTERMEDIATE * INTERMEDIATE_TO_MOTOR) / (MAX_MOTOR_RPM);
//
        double safety = 1;

        /*
        Get final motor RPMs
        Put a negative sign in two calculations to make sure modules and wheels are turning coherently
         */
        double m0RPM =
                ((wheelModifier * leftInvert * leftWheelRPM * WHEEL_TO_MODULE * COMMON_GEAR_RATIO)
                        + leftSign * leftRPM * COMMON_GEAR_RATIO) * safety;
        double m1RPM =
                (-(wheelModifier * leftInvert * leftWheelRPM * WHEEL_TO_MODULE * COMMON_GEAR_RATIO)
                        + leftSign * leftRPM * COMMON_GEAR_RATIO) * safety;
        double m2RPM =
                ((wheelModifier * rightInvert * rightWheelRPM * WHEEL_TO_MODULE * COMMON_GEAR_RATIO)
                        + rightSign * rightRPM * COMMON_GEAR_RATIO) * safety;
        double m3RPM =
                (-(wheelModifier * rightInvert * rightWheelRPM * WHEEL_TO_MODULE * COMMON_GEAR_RATIO)
                        + rightSign * rightRPM * COMMON_GEAR_RATIO) * safety;

        //RPM to Radians Per Second conversion
        double RPM_to_RadPerSec = TAU / 60;

        //set motor velocities
        if (givePower) {
            m0.setVelocity(m0RPM * RPM_to_RadPerSec, AngleUnit.RADIANS);
            m1.setVelocity(m1RPM * RPM_to_RadPerSec, AngleUnit.RADIANS);
            m2.setVelocity(m2RPM * RPM_to_RadPerSec, AngleUnit.RADIANS);
            m3.setVelocity(m3RPM * RPM_to_RadPerSec, AngleUnit.RADIANS);
        } else {
            m0.setPower(0);
            m1.setPower(0);
            m2.setPower(0);
            m3.setPower(0);
        }

        prevLeftAlpha = leftAlpha;
        prevRightAlpha = rightAlpha;
        prevRotAlpha = rotAlpha;
        prevTime = currentTime;
    }

//    public double getIMUAngle() {
//        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        return angles.firstAngle;
//    }


    public Pose2d globalPosition() {
        return localizer.getGlobalPose();
    }

    public void updateBulkData(RevBulkData controlHubData, RevBulkData exHubData) {
        cdata = controlHubData;
        edata = exHubData;
    }

    public void update() {
        m0Encoder = m0Offset - cdata.getMotorCurrentPosition(0);
        m1Encoder = m1Offset + cdata.getMotorCurrentPosition(1);
        m2Encoder = m2Offset - edata.getMotorCurrentPosition(0);
        m3Encoder = m3Offset + edata.getMotorCurrentPosition(1);

        leftModulePhi = myMath.NEWwrapTo180(-360 * ((m0Encoder + m1Encoder) / 2) / TICKS_PER_MODULE_REV);
        rightModulePhi = myMath.NEWwrapTo180(-360 * ((m2Encoder + m3Encoder) / 2) / TICKS_PER_MODULE_REV);

        rawLeftModulePhi = -360 * ((m0Encoder + m1Encoder) / 2) / TICKS_PER_MODULE_REV;
    }

    public void rotateLeftModule(double angle) {
        double error = angle - Math.abs(leftModulePhi);

        if (Math.abs(error) < 5) {
            m0.setPower(0);
            m1.setPower(0);
        } else {
            m0.setPower(error * 1 / 160);
            m1.setPower(error * 1 / 160);
        }
    }

    public void rotateRightModule(double angle) {
//        double error = angle - Math.abs(rightModulePhi);
//
//        if (Math.abs(error) < 5) {
//            m2.setPower(0);
//            m3.setPower(0);
//        } else {
//            m2.setPower(error);
//            m3.setPower(error);
//        }
        m2.setPower(0.5);
        m3.setPower(-0.5);
    }

    /*
    Used to enable auto aim
     */
    public void setAutoAim(boolean useAutoAim) {
        autoAim = useAutoAim;
    }

    /*
    stop the robot
     */
    public void stopRobot() {
        m0.setPower(0);
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
    }

    public void spinDiffy(double targetAngle) {
//        motor0.setVelocity(10, AngleUnit.DEGREES);
//        motor1.setVelocity(10, AngleUnit.DEGREES);
        m0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        update();
        double controller_output = modulePID.calculate(leftModulePhi, targetAngle);
        m0.setPower(controller_output);
        m1.setPower(controller_output);

    }

    public void turn(double power) {
        m0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        update();

        m0.setPower(power/2);
        m1.setPower(-power/2);
        m2.setPower(-power/2);
        m3.setPower(power/2);
    }

    public double getLeftModulePhi() {
        return leftModulePhi;
    }

    public double getRawLeftPhi() {
        return rawLeftModulePhi;
    }
    public double getRightModulePhi() {
        return rightModulePhi;
    }

    public double getLeftTheta() {
        return leftModuleTheta;
    }

    public double getTransLeftTheta() {
        return transLeftTheta;
    }

    public double getleftWheelRPM() {
        return leftWheelRPM;
    }

    public double getRightWheelRPM() {
        return rightWheelRPM;
    }

    public void resetEncoders() {
        m0Encoder = 0;
        m1Encoder = 0;
        m2Encoder = 0;
        m3Encoder = 0;

        m0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m0Offset = 0;
        m1Offset = 0;
        m2Offset = 0;
        m3Offset = 0;
    }

    public void setEncoderOffsets(double m0Enc, double m1Enc, double m2Enc, double m3Enc) {
        m0Offset = m0Enc;
        m1Offset = m1Enc;
        m2Offset = m2Enc;
        m3Offset = m3Enc;
    }


}