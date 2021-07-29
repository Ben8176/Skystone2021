package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.localization.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.LeftModule;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.Vector2d;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.Arrays;
import java.util.List;

@Config
public class Robot {

    public static double positionKp = 0.0032;
    public static double positionKi = 0.001;
    public static double positionKd = 0;

    public BNO055IMU imu;
    public ExpansionHubEx controlHub, exHub;

    public ExpansionHubMotor motor0, motor1, motor2, motor3;
    public ExpansionHubMotor leftOdom, flywheelB, flywheelA, perpOdom;

    MathUtil myMath = new MathUtil();
    ElapsedTime robotTimer = new ElapsedTime();
    PIDController positionController = new PIDController();

    public LeftModule lmodule;

    public boolean initComplete;

    public ThreeWheelLocalizer localizer;
    public Drivetrain drive;

    //bulk hub data
    RevBulkData controlData, exData;
    Telemetry telemetry;

//    private boolean initialErrorFlag = true;
    private double targetAngle = 0;
    private double targetHeading = 0;
    private double translationPower = 0;
    private boolean trajectoryEnd = false;
    private double headingError = 0;
    private final double kSigma = 1.05;
    double initialError = 0;
    private double errorMagnitude = 0;

    public static Pose2d autoFinalPose = new Pose2d();
    private Pose2d initialPose = new Pose2d();

    public Robot(HardwareMap hwmap, Pose2d startPose, Telemetry telemetry) {
        //------------------------------------------------------------------
        this.telemetry = telemetry;

        List<LynxModule> allHubs = hwmap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        }

        //Initialize all IMU stuff
        imu = hwmap.get(BNO055IMU.class, "IMU");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        //Drivetrain motors
        motor0 = hwmap.get(ExpansionHubMotor.class, "motor0");
        motor1 = hwmap.get(ExpansionHubMotor.class,"motor1");
        motor2 = hwmap.get(ExpansionHubMotor.class, "motor2");
        motor3 = hwmap.get(ExpansionHubMotor.class,"motor3");
        //flywheelA has RIGHT ODOMETRY AS ITS ENCODER!!!
        flywheelA = hwmap.get(ExpansionHubMotor.class, "flywheelA");
        leftOdom = hwmap.get(ExpansionHubMotor.class, "intake");
        perpOdom = hwmap.get(ExpansionHubMotor.class, "transferMotor");


        controlHub = hwmap.get(ExpansionHubEx.class, "Expansion Hub 2");
        exHub = hwmap.get(ExpansionHubEx.class, "Expansion Hub 3");

        //setup three wheel localizer
        localizer = new ThreeWheelLocalizer();
        localizer.setStartPose(startPose);

        //adjust the max rpm of "Run Using Encoders" motors
        //RESET ALL ENCODERS ON THE ROBOT! VERY IMPORTANT!
        List<ExpansionHubMotor> RUE_motors = Arrays.asList(motor0,motor1,motor2,motor3,perpOdom);
        for (DcMotor motor: RUE_motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);

            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        //reset encoders and set runmodes for other motors
        List<ExpansionHubMotor> RWE_motors = Arrays.asList(flywheelA);
        for (ExpansionHubMotor motor : RWE_motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        //set zero power behavior for the drive motors
        List<ExpansionHubMotor> driveList = Arrays.asList(motor0,motor1,motor2,motor3);
        for(DcMotor motor: driveList) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        //import the motors into the drivetrain subsystem
        drive = new Drivetrain
                (telemetry, motor0, motor1, motor2, motor3, localizer);
        lmodule = new LeftModule(motor0, motor1);

    }

    public void initAll() {
        initComplete = false;
//        vision.initVision();
        initMotors();
        initServos();
        initComplete = true;
    }

    public void initMotors() {

//        motor0.setDirection(DcMotorSimple.Direction.REVERSE);
//        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor3.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelA.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void initServos() {
    }

    public void driveToPointPID(Pose2d targetPose, double powerModifier) {
        errorMagnitude = Math.hypot(targetPose.getX() - getPosition().getX(), targetPose.getY() - getPosition().getY());
        targetAngle = Math.toDegrees(Math.atan2(targetPose.getY() - getPosition().getY(), targetPose.getX() - getPosition().getX()));
        targetHeading = targetPose.getTheta() - getPosition().getTheta();


        if (errorMagnitude > 1) {
            translationPower = positionController.calculateWithError(errorMagnitude, positionKp, positionKi, positionKd);
            trajectoryEnd = false;
        }
        else {
            translationPower = 0;
            trajectoryEnd = true;
        }

        drive.updateSwerve(targetHeading, 1, targetAngle, translationPower * powerModifier, true);

    }

    /*
    control the robot using the gamepad sticks
     */
    public void driveTeleOp(Vector2d rotStick, Vector2d transStick, double percentPower) {
        drive.updateSwerve(Math.toDegrees(rotStick.getAngle()), rotStick.magnitude(),
                Math.toDegrees(transStick.getAngle()),
                transStick.magnitude() * percentPower, true);
    }

    public void stop() {
        drive.stopRobot();
    }

    /*
    Method to drive to a point and slowdown, independent control of heading
     */
    public void driveToPoint(Pose2d targetPose, double powerModifier, boolean givePower) {
        errorMagnitude = Math.hypot(targetPose.getX() - getPosition().getX(), targetPose.getY() - getPosition().getY());
        targetAngle = Math.toDegrees(Math.atan2(targetPose.getY() - getPosition().getY(), targetPose.getX() - getPosition().getX()));
        targetHeading = targetPose.getTheta() - getPosition().getTheta();
        if (errorMagnitude > 1) {
            translationPower = (2 * Math.pow(kSigma, errorMagnitude)) / (Math.pow(kSigma, errorMagnitude) + 1) - 1;
            trajectoryEnd = false;
        }
        else {
            translationPower = 0;
            trajectoryEnd = true;
        }
        drive.updateSwerve(targetHeading, 1, targetAngle, translationPower * powerModifier, givePower);
    }

    /*
    go to point function to start with metered acceleration and gun it the rest of the way, no slowdown.
    good for connecting trajectories with driveToPoint() method.
     */
    public void startSmoothEndHard(Pose2d targetPose, double powerModifier) {
        errorMagnitude = Math.hypot(targetPose.getX() - getPosition().getX(), targetPose.getY() - getPosition().getY());
        //only runs once, right after the previous trajectory finished and set trajectoryEnd to true
        //after this if statement, trajectoryEnd is set to false because we are running the trajectory
        if (trajectoryEnd) {
            initialError = errorMagnitude;
        }
        targetAngle = Math.toDegrees(Math.atan2(targetPose.getY() - getPosition().getY(), targetPose.getX() - getPosition().getX()));
        targetHeading = targetPose.getTheta() - getPosition().getTheta();
        if (errorMagnitude > 1) {
            if (errorMagnitude < initialError / 2) {
                translationPower = powerModifier;
            }
            else {
                translationPower = myMath.powerMotionProfile(errorMagnitude, initialError);
            }
            trajectoryEnd = false;
        }
        else {
            translationPower = 0;
            trajectoryEnd = true;
        }
        drive.updateSwerve(targetHeading,1, targetAngle, translationPower * powerModifier, true);
    }
    /*
    Drive to point with full motion profiling, metered acceleration and decel
     */
    public void driveToPointMP(Pose2d targetPose, double powerModifier) {
        errorMagnitude = Math.hypot(targetPose.getX() - getPosition().getX(), targetPose.getY() - getPosition().getY());
        //only runs once, right after the previous trajectory finished and set trajectoryEnd to true
        //after this if statement, trajectoryEnd is set to false because we are running the trajectory
        if (trajectoryEnd) {
            initialError = errorMagnitude;
        }
        targetAngle = Math.toDegrees(Math.atan2(targetPose.getY() - getPosition().getY(), targetPose.getX() - getPosition().getX()));
        targetHeading = targetPose.getTheta() - getPosition().getTheta();
        if (errorMagnitude > 1) {
            translationPower = myMath.powerMotionProfile(errorMagnitude, initialError);
            trajectoryEnd = false;
        }
        else {
            translationPower = 0;
            trajectoryEnd = true;
        }
        drive.updateSwerve(targetHeading,1, targetAngle, translationPower * powerModifier, true);
    }
    /*
    Method to turn on a point to any desired degree (NO DECIMALS!)
     */
    public void turn(double targetHeading) {
        this.targetHeading = targetHeading;
        drive.updateSwerve(targetHeading, 1, 0, 0, true);
        headingError = targetHeading - Math.toDegrees(getPosition().getTheta());
        trajectoryEnd = Math.abs(headingError) < 0.5; //test if trajectory is done
    }

    /*
    Update all the data on the robot, including odometry
     */
    public void update() {
        //get new data from hubs
        updateData();
        //update position
        updatePosition();
        //update the bulk data for the drivetrain encoders
        drive.updateBulkData(controlData, exData);

        drive.update();
    }

    /*
    Update odometry global position
     */
    public void updatePosition() {
        localizer.update(controlData, exData);
    }

    /*
    Updates the control hub and expansion hub data readings
    Very important dont touch!
     */
    public void updateData() {
        controlData = controlHub.getBulkInputData();
        exData = exHub.getBulkInputData();
    }

    /*
    Get the current pose of the robot from odometry localization
    EXTREMELY important! Don't mess with
     */
    public Pose2d getPosition() {
        return localizer.getGlobalPose();
    }


}