package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.util.Constants.ANCHOR;
import static org.firstinspires.ftc.teamcode.util.Constants.BLUE;
import static org.firstinspires.ftc.teamcode.util.Constants.BLUE_LOWER;
import static org.firstinspires.ftc.teamcode.util.Constants.BLUE_UPPER;
import static org.firstinspires.ftc.teamcode.util.Constants.BLUR_SIZE;
import static org.firstinspires.ftc.teamcode.util.Constants.ERODE_DILATE_ITERATIONS;
import static org.firstinspires.ftc.teamcode.util.Constants.INVALID_POINT;
import static org.firstinspires.ftc.teamcode.util.Constants.MIN_GOAL_AREA;
import static org.firstinspires.ftc.teamcode.util.Constants.MIN_POWERSHOT_AREA;
import static org.firstinspires.ftc.teamcode.util.Constants.POWERSHOT_DIMENSIONS;
import static org.firstinspires.ftc.teamcode.util.Constants.POWERSHOT_OFFSET;
import static org.firstinspires.ftc.teamcode.util.Constants.RED;
import static org.firstinspires.ftc.teamcode.util.Constants.RED_LOWER_1;
import static org.firstinspires.ftc.teamcode.util.Constants.RED_LOWER_2;
import static org.firstinspires.ftc.teamcode.util.Constants.RED_UPPER_1;
import static org.firstinspires.ftc.teamcode.util.Constants.RED_UPPER_2;
import static org.firstinspires.ftc.teamcode.util.Constants.STRUCTURING_ELEMENT;
import static org.firstinspires.ftc.teamcode.util.Constants.WHITE;
import static org.firstinspires.ftc.teamcode.util.OpenCVUtil.getLargestContour;
import static org.firstinspires.ftc.teamcode.util.OpenCVUtil.getLargestContours;

// Class for the pipeline that is used to detect the goals and powershots
public class TargetingPipeline extends OpenCvPipeline {
    Mat blurred = new Mat();
    Mat hsv = new Mat();
    Mat redMask1 = new Mat();
    Mat redMask2 = new Mat();
    Mat redMask = new Mat();
    Mat blueMask = new Mat();

    private Detection red;
    private Detection blue;
    private PowershotDetection redPowershots;
    private PowershotDetection bluePowershots;

    // Init
    @Override
    public void init(Mat input) {
        red = new Detection(input.size(), MIN_GOAL_AREA);
        blue = new Detection(input.size(), MIN_GOAL_AREA);
        redPowershots = new PowershotDetection(input.size(), MIN_POWERSHOT_AREA);
        bluePowershots = new PowershotDetection(input.size(), MIN_POWERSHOT_AREA);
    }

    // Process each frame that is received from the webcam
    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.GaussianBlur(input, blurred, BLUR_SIZE, 0);
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);

        updateRed(input);
        updateBlue(input);
        updateRedPowershots(input);
        updateBluePowershots(input);

        return input;
    }

    // Update the Red Goal Detection
    private void updateRed(Mat input) {
        // take pixels that are in the color range and put them into a mask, eroding and dilating them to remove white noise
        Core.inRange(hsv, RED_LOWER_1, RED_UPPER_1, redMask1);
        Core.inRange(hsv, RED_LOWER_2, RED_UPPER_2, redMask2);
        Core.add(redMask1, redMask2, redMask);
        Imgproc.erode(redMask, redMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);
        Imgproc.dilate(redMask, redMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);

        // set the largest detection that was found to be the Red Goal detection
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(redMask, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        red.setContour(getLargestContour(contours));

        // draw the Red Goal detection
        red.draw(input, RED);
    }

    // Update the Blue Goal Detection
    private void updateBlue(Mat input) {
        // take pixels that are in the color range and put them into a mask, eroding and dilating them to remove white noise
        Core.inRange(hsv, BLUE_LOWER, BLUE_UPPER, blueMask);
        Imgproc.erode(blueMask, blueMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);
        Imgproc.dilate(blueMask, blueMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);

        // set the largest detection that was found to be the Red Goal detection
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(blueMask, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        blue.setContour(getLargestContour(contours));

        // draw the Blue Goal detection
        blue.draw(input, BLUE);
    }

    // Update the Red Powershot Detection
    private void updateRedPowershots(Mat input) {
        // check if the Red Goal is in sight (the powershots are based off of where the red goal is)
        if (!this.red.isValid()) {
            return;
        }

        // create a rectangle based off of the goal of where to look for the powershots
        Rect powershotRect = new Rect(
                (int) (red.getBottomLeftCornerPx().x - POWERSHOT_DIMENSIONS.width + POWERSHOT_OFFSET.x),
                (int) (red.getBottomLeftCornerPx().y + POWERSHOT_OFFSET.y),
                (int) POWERSHOT_DIMENSIONS.width,
                (int) POWERSHOT_DIMENSIONS.height
        );

        // take pixels that are in the color range and put them into a mask
        Core.inRange(hsv, RED_LOWER_1, RED_UPPER_1, redMask1);
        Core.inRange(hsv, RED_LOWER_2, RED_UPPER_2, redMask2);
        Core.add(redMask1, redMask2, redMask);

        // make a new mask that will be used to find the contours of the Powershots
        Mat powershotMask = new Mat(hsv.size(), CvType.CV_8U);
        Imgproc.rectangle(powershotMask, powershotRect, WHITE, -1);
        Core.bitwise_and(redMask, powershotMask, powershotMask);

        // set the largest detection that was found to be the Powershots
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(powershotMask, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        redPowershots.setContours(getLargestContours(contours, 3));

        // draw the Powershot detection as well as where was looked for it on the screen
        Imgproc.rectangle(input, powershotRect, WHITE, 2);
        redPowershots.draw(input, WHITE);
    }

    // Update the Blue Powershot Detection
    private void updateBluePowershots(Mat input) {
        // check if the Blue Goal is in sight (the powershots are based off of where the blue goal is)
        if (!this.blue.isValid()) {
            return;
        }

        // create a rectangle based off of the goal of where to look for the powershots
        Rect powershotRect = new Rect(
                (int) (blue.getBottomRightCornerPx().x + POWERSHOT_OFFSET.x),
                (int) (blue.getBottomRightCornerPx().y + POWERSHOT_OFFSET.y),
                (int) POWERSHOT_DIMENSIONS.width,
                (int) POWERSHOT_DIMENSIONS.height
        );

        // take pixels that are in the color range and put them into a mask
        Core.inRange(hsv, BLUE_LOWER, BLUE_UPPER, blueMask);

        // make a new mask that will be used to find the contours of the Powershots
        Mat powershotMask = new Mat(hsv.size(), CvType.CV_8U);
        Imgproc.rectangle(powershotMask, powershotRect, WHITE, -1);
        Core.bitwise_and(blueMask, powershotMask, powershotMask);

        // set the largest detection that was found to be the Powershots
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(powershotMask, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        bluePowershots.setContours(getLargestContours(contours, 3));

        // draw the Powershot detection as well as where was looked for it on the screen
        Imgproc.rectangle(input, powershotRect, WHITE, 2);
        bluePowershots.draw(input, WHITE);
    }

    // Get the center of whatever goal is in sight
    public Point getCenterOfLargestContour() {
        if (blue.getArea() > red.getArea()) {
            return blue.getCenter();
        }

        if (red.getArea() > blue.getArea()) {
            return red.getCenter();
        }

        return INVALID_POINT;
    }

    // Get the area of whatever goal is in sight
    public double getAreaOfLargestContour() {
        return Math.max(blue.getArea(), red.getArea());
    }

    // Get the Red Goal Detection
    public Detection getRed() {
        return red;
    }

    // Get the Blue Goal Detection
    public Detection getBlue() {
        return blue;
    }

    // Get the Powershot Detection
    public PowershotDetection getRedPowershots() {
        return redPowershots;
    }

    // Get the Powershot Detection
    public PowershotDetection getBluePowershots() {
        return bluePowershots;
    }
}
