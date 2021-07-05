package org.firstinspires.ftc.teamcode.vision;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class ContourPipeline extends OpenCvPipeline {

    Scalar lowerOrange = new Scalar(0.0, 90.0, 0.0);
    Scalar upperOrange = new Scalar(255.0, 200.0, 112.0);

    final int CAMERA_WIDTH = 320;
    final int HORIZON = 100;
    final double MIN_WIDTH = 50.0;
    final double FOUR_STACK_RATIO = 0.7;

    Rect maxRect = new Rect();


    public enum Height {
        ZERO,
        ONE,
        FOUR
    }

    Height height = Height.ZERO;


    @Override
    public void init(Mat input) {
        /* Executed once, when the pipeline is selected */
    }

    Mat mat = new Mat();
    Mat mask = new Mat();
    Mat ret = new Mat();
    Mat hierarchy = new Mat();
    ArrayList<MatOfPoint> contours = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {
        /* Executed each frame, the returned mat will be the one displayed */
        /* Processing and detection stuff */

        ret.release();
        /*
         * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
         * will only dereference to the same image for the duration of this particular
         * invocation of this method. That is, if for some reason you'd like to save a copy
         * of this particular frame for later use, you will need to either clone it or copy
         * it to another Mat.
         */

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);

        mask = new Mat(mat.rows(), mat.cols(), CvType.CV_8UC1);
        Core.inRange(mat, lowerOrange, upperOrange, mask);

        Core.bitwise_and(input, input, ret, mask);


        Imgproc.erode(mask, mask, new Mat(), new Point(), 3);

        Imgproc.GaussianBlur(mask, mask, new Size(5, 15.0), 0);


        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
        Imgproc.drawContours(ret, contours, -1, new Scalar(0, 255, 0), 1);


        double maxWidth = 0;
        for (MatOfPoint points : contours) {
            Mat copy = new MatOfPoint2f(points.toArray());
            Rect rect = Imgproc.boundingRect(copy);

            int w = rect.width;
            if (w > maxWidth && rect.y > HORIZON) {
                maxWidth = w;
                maxRect = rect;
            }
        }

        Imgproc.rectangle(ret, maxRect, new Scalar(0.0, 0.0, 255.0), 2);

        Imgproc.line(
                ret,
                new Point(
                        .0,
                        HORIZON
                ),
                new Point(
                        CAMERA_WIDTH,
                        HORIZON
                ),
                new Scalar(
                        255.0,
                        .0,
                        255.0)
        );

        double ratio = (double) maxRect.height / maxRect.width;

        if (maxWidth != 0) {
            if (ratio < FOUR_STACK_RATIO) {
                height = Height.ONE;
            }
            else {
                height = Height.FOUR;
            }
        }
        else {
            height = Height.ZERO;
        }

        return ret;

        // Return the input mat
        // (Or a new, processed mat)
    }

    @Override
    public void onViewportTapped() {
        /*
         * Executed everytime when the pipeline view is tapped/clicked.
         * This is executed from the UI thread, so whatever we do here
         * we must do it quickly.
         */
    }

    /*
    0 zone A ZERO
    1 zone B ONE
    2 zone C FOUR
     */
    public int getZone() {
        return height.ordinal();
    }

}
