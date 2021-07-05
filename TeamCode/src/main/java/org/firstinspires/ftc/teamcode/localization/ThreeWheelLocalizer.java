package org.firstinspires.ftc.teamcode.localization;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.Vector2d;
import org.openftc.revextensions2.RevBulkData;

public class ThreeWheelLocalizer {

    //in CM
    private final double TRACK_WIDTH = 21.9;
    private final double PARALLEL_X_OFFSET = -14.6;
    private final double PERP_X_OFFSET = -18.5;
    private final double PERP_Y_OFFSET = 2;
    private final int LEFT_PARALLEL_PORT = 3;
    private final int RIGHT_PARALLEL_PORT = 2;
    private final int PERPENDICULAR_PORT = 2;

    private double TICKS_PER_REV = 8192;
    private double WHEEL_RADIUS = 2.4;

    private DecompositionSolver forwardSolver;
    public PoseExponential poseExponential = new PoseExponential();

    public Pose2d currentPosition = new Pose2d(0,0,0);

    Pose2d[] wheelPoses = {
            new Pose2d(PARALLEL_X_OFFSET, 11.6, 0), //left
            new Pose2d(PARALLEL_X_OFFSET, -10.3, 0), //right
            new Pose2d(PERP_X_OFFSET, PERP_Y_OFFSET, Math.toRadians(90)) //perp
    };

    //instantiate prevWheelPositions as empty double array with 3 fields
    double[] prevWheelPositions = {0,0,0};
    double leftticks = 0;
    double rightticks = 0;
    double perpticks = 0;

    /**
     * Setup inverse kinematics for wheel positions to inches travelled
     */
    public ThreeWheelLocalizer() {

        Array2DRowRealMatrix inverseMatrix = new Array2DRowRealMatrix(3,3);

        //populate a matrix with deadwheel poses
        Vector2d orientationVec;
        Vector2d positionVec;
        for (int i = 0 ; i < 3; i++) {
            orientationVec = wheelPoses[i].headingVec();
            positionVec = wheelPoses[i].vec();
            inverseMatrix.setEntry(i, 0, orientationVec.getX());
            inverseMatrix.setEntry(i, 1, orientationVec.getY());
            inverseMatrix.setEntry(i, 2,
                    positionVec.getX() * orientationVec.getY() - positionVec.getY() * orientationVec.getX());
        }

        forwardSolver = new LUDecomposition(inverseMatrix).getSolver();

        if (!forwardSolver.isNonSingular()) {
            throw new IllegalArgumentException("The specified configuration cannot support full localization");
        }


        //set our currentposition to the start pose declared in the opmode
//        currentPosition = new Pose2d(startPose.getX(), startPose.getY(), startPose.getTheta());
    }

    public double encoderTicksToCM(double ticks) {
        return 2 * Math.PI * WHEEL_RADIUS * ticks / TICKS_PER_REV;
    }

    /*
    Calculate the robot pose x, y, and heading changes from the wheel changes
     */
    public Pose2d calculatePoseDeltas(double[] deltas) {
        RealMatrix m = MatrixUtils.createRealMatrix(new double[][] {deltas});

        RealMatrix rawPoseDelta = forwardSolver.solve(m.transpose());

        return new Pose2d(
                rawPoseDelta.getEntry(0, 0),
                rawPoseDelta.getEntry(1, 0),
                rawPoseDelta.getEntry(2, 0)
        );
    }

    public void setStartPose(Pose2d startPose) {
        currentPosition = startPose;
    }

    /*
    Update the robots pose on the field
     */
    public void update(RevBulkData cdata, RevBulkData edata) {
        leftticks = -cdata.getMotorCurrentPosition(LEFT_PARALLEL_PORT);
        rightticks = -edata.getMotorCurrentPosition(RIGHT_PARALLEL_PORT);
        perpticks = cdata.getMotorCurrentPosition(PERPENDICULAR_PORT);

        //get the wheel changes this loop (in CM)
        double[] wheelDeltas = {
                encoderTicksToCM(leftticks - prevWheelPositions[0]),
                encoderTicksToCM(rightticks - prevWheelPositions[1]),
                encoderTicksToCM(perpticks - prevWheelPositions[2])
        };

        //Calculate the robot pose change from the wheel deltas
        Pose2d robotPoseDeltas = calculatePoseDeltas(wheelDeltas);
        //update current pose through pose exponential: add global pose change to previous global pose
        currentPosition = poseExponential.globalOdometryUpdate(currentPosition, robotPoseDeltas);
        //store wheel positions for changes next loop
        prevWheelPositions[0] = leftticks;
        prevWheelPositions[1] = rightticks;
        prevWheelPositions[2] = perpticks;
    }

    public Pose2d getGlobalPose() {
        return currentPosition;
    }

    public double[] odomTicks() {
        return new double[] {
                leftticks,
                rightticks,
                perpticks
        };
    }

}
