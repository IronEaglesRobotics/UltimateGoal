package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.CVHelpers.BLUE;
import static org.firstinspires.ftc.teamcode.CVHelpers.RED;
import static org.firstinspires.ftc.teamcode.CVHelpers.getLargestContour;

class CVPipeline extends OpenCvPipeline
{
    Mat blurred = new Mat();
    Mat hsv = new Mat();
    Mat redMask1 = new Mat();
    Mat redMask2 = new Mat();
    Mat redMask = new Mat();
    Mat blueMask = new Mat();

    final Mat STRUCTURING_ELEMENT = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size((2 * 2) + 1, (2 * 2) + 1));
    final Point ANCHOR = new Point((STRUCTURING_ELEMENT.cols() / 2f), STRUCTURING_ELEMENT.rows() / 2f);
    final double MIN_AREA = 0.05;
    final Scalar RED_LOWER_1 = new Scalar(0, 90, 90);
    final Scalar RED_UPPER_1 = new Scalar(2, 255, 255);
    final Scalar RED_LOWER_2 = new Scalar(165, 90, 90);
    final Scalar RED_UPPER_2 = new Scalar(180, 255, 255);
    final Scalar BLUE_LOWER = new Scalar(75, 85, 100);
    final Scalar BLUE_UPPER = new Scalar(120, 255, 255);
    final int ERODE_DILATE_ITERATIONS = 2;
    final Size BLUR_SIZE = new Size(11, 11);

    private Detection red;
    private Detection blue;

    @Override
    public void init(Mat input) {
        red = new Detection(input.size(), MIN_AREA);
        blue = new Detection(input.size(), MIN_AREA);
    }

    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.GaussianBlur(input, blurred, BLUR_SIZE, 0);
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);

        updateRed();
        red.draw(input, RED);

        updateBlue();
        blue.draw(input, BLUE);

        return input;
    }

    private void updateRed() {
        // Get the mask for RED objects. Note that since we're in HSV color space, red appears at
        // both the lower and upper end of the H spectrum. Thus, we have two separate red masks
        // which we then merge into one.
        Core.inRange(hsv, RED_LOWER_1, RED_UPPER_1, redMask1);
        Core.inRange(hsv, RED_LOWER_2, RED_UPPER_2, redMask2);
        Core.add(redMask1, redMask2, redMask);
        Imgproc.erode(redMask, redMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);
        Imgproc.dilate(redMask, redMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(redMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        red.setContour(getLargestContour(contours));
    }

    private void updateBlue() {
        // Get the mask for BLUE objects. Note that because blue is isolated in the H spectrum, we
        // do not need two separate sets of lower and upper values.
        Core.inRange(hsv, BLUE_LOWER, BLUE_UPPER, blueMask);
        Imgproc.erode(blueMask, blueMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);
        Imgproc.dilate(blueMask, blueMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(blueMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        blue.setContour(getLargestContour(contours));
    }

    public Point getCenterOfLargestContour() {
        if (blue.getArea() > red.getArea()) {
            return blue.getCenter();
        }

        if (red.getArea() > blue.getArea()) {
            return red.getCenter();
        }

        return Detection.INVALID_POINT;
    }

    public double getAreaOfLargestContour() {
        return Math.max(blue.getArea(), red.getArea());
    }

    public Detection getRed() {
        return red;
    }

    public Detection getBlue() {
        return blue;
    }
}
