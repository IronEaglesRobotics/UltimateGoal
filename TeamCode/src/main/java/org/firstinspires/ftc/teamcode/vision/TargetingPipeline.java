package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.util.OpenCVUtil;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.util.Configurables.CAMERA_BLUE_GOAL_LOWER;
import static org.firstinspires.ftc.teamcode.util.Configurables.CAMERA_BLUE_GOAL_UPPER;
import static org.firstinspires.ftc.teamcode.util.Configurables.CAMERA_BLUE_POWERSHOT_LOWER;
import static org.firstinspires.ftc.teamcode.util.Configurables.CAMERA_BLUE_POWERSHOT_UPPER;
import static org.firstinspires.ftc.teamcode.util.Configurables.CAMERA_RED_GOAL_LOWER;
import static org.firstinspires.ftc.teamcode.util.Configurables.CAMERA_RED_GOAL_UPPER;
import static org.firstinspires.ftc.teamcode.util.Configurables.CAMERA_RED_POWERSHOT_LOWER;
import static org.firstinspires.ftc.teamcode.util.Configurables.CAMERA_RED_POWERSHOT_UPPER;
import static org.firstinspires.ftc.teamcode.util.Configurables.CAMERA_WHITE_LOWER;
import static org.firstinspires.ftc.teamcode.util.Configurables.CAMERA_WHITE_UPPER;
import static org.firstinspires.ftc.teamcode.util.Configurables.CV_MAX_GOAL_AREA;
import static org.firstinspires.ftc.teamcode.util.Configurables.CV_MAX_POWERSHOT_AREA;
import static org.firstinspires.ftc.teamcode.util.Configurables.CV_MIN_GOAL_AREA;
import static org.firstinspires.ftc.teamcode.util.Configurables.CV_MIN_POWERSHOT_AREA;
import static org.firstinspires.ftc.teamcode.util.Configurables.CV_POWERSHOT_DIMENSIONS;
import static org.firstinspires.ftc.teamcode.util.Configurables.CV_POWERSHOT_OFFSET;
import static org.firstinspires.ftc.teamcode.util.Constants.ANCHOR;
import static org.firstinspires.ftc.teamcode.util.Constants.BLUE;
import static org.firstinspires.ftc.teamcode.util.Constants.ERODE_DILATE_ITERATIONS;
import static org.firstinspires.ftc.teamcode.util.Constants.GRAY;
import static org.firstinspires.ftc.teamcode.util.Constants.INVALID_POINT;
import static org.firstinspires.ftc.teamcode.util.Constants.RED;
import static org.firstinspires.ftc.teamcode.util.Constants.STRUCTURING_ELEMENT;
import static org.firstinspires.ftc.teamcode.util.Constants.WHITE;
import static org.firstinspires.ftc.teamcode.util.OpenCVUtil.getConfidenceContour;
import static org.firstinspires.ftc.teamcode.util.OpenCVUtil.getHighGoalContour;
import static org.firstinspires.ftc.teamcode.util.OpenCVUtil.getLargestContours;


// Class for the pipeline that is used to detect the goals and powershots
public class TargetingPipeline extends OpenCvPipeline {
    Mat blurred = new Mat();
    Mat hsv = new Mat();
    Mat redMask1 = new Mat();
    Mat redMask2 = new Mat();
    Mat redMask = new Mat();
    Mat blueMask = new Mat();
    Mat blackMask = new Mat();
    Mat whiteMask = new Mat();
    Scalar redGoalLower1;
    Scalar redGoalUpper1;
    Scalar redGoalLower2;
    Scalar redGoalUpper2;
    Scalar redPowershotLower1;
    Scalar redPowershotUpper1;
    Scalar redPowershotLower2;
    Scalar redPowershotUpper2;

    private Detection red;
    private Detection blue;
    private PowershotDetection redPowershots;
    private PowershotDetection bluePowershots;

    // Init
    @Override
    public void init(Mat input) {
        red = new Detection(input.size(), CV_MIN_GOAL_AREA);
        blue = new Detection(input.size(), CV_MIN_GOAL_AREA);
        redPowershots = new PowershotDetection(input.size(), CV_MIN_POWERSHOT_AREA);
        bluePowershots = new PowershotDetection(input.size(), CV_MIN_POWERSHOT_AREA);
    }

    // Process each frame that is received from the webcam
    @Override
    public Mat processFrame(Mat input)
    {
//        Imgproc.GaussianBlur(input, blurred, BLUR_SIZE, 0);
//        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        updateWhite(input);
//        updateRed(input);
//        updateBlue(input);
        updateRedPowershots(input);
        updateBluePowershots(input);

        return input;
    }

    private void updateWhite(Mat input) {
        Core.inRange(hsv , new Scalar(CAMERA_WHITE_LOWER.get()), new Scalar(CAMERA_WHITE_UPPER.get()), whiteMask);
        Imgproc.erode(whiteMask, whiteMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);
        Imgproc.dilate(whiteMask, whiteMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);

        ArrayList<MatOfPoint> contoursWhite = new ArrayList<>();
        Imgproc.findContours(whiteMask, contoursWhite, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < contoursWhite.size(); i++) {
            Detection newDetection = new Detection(input.size(),0.01);
            newDetection.setContour(contoursWhite.get(i));
            newDetection.draw(input, WHITE);
        }
        MatOfPoint contour = getConfidenceContour(contoursWhite, hsv);
        if (contour != null) {
            red.setContour(contour);
        }
//        red.setContour(getConfidenceContour(contoursWhite, hsv));

        // draw the Red Goal detection
        red.fill(input, GRAY);
    }
    // Update the Red Goal Detection
    private void updateRed(Mat input) {
        // take pixels that are in the color range and put them into a mask, eroding and dilating them to remove white noise
        redGoalLower1 = new Scalar(CAMERA_RED_GOAL_LOWER.getH(), CAMERA_RED_GOAL_LOWER.getS(), CAMERA_RED_GOAL_LOWER.getV());
        redGoalUpper1 = new Scalar(180, CAMERA_RED_GOAL_UPPER.getS(), CAMERA_RED_GOAL_UPPER.getV());
        redGoalLower2 = new Scalar(0, CAMERA_RED_GOAL_LOWER.getS(), CAMERA_RED_GOAL_LOWER.getV());
        redGoalUpper2 = new Scalar(CAMERA_RED_GOAL_UPPER.getH(), CAMERA_RED_GOAL_UPPER.getS(), CAMERA_RED_GOAL_UPPER.getV());
        Core.inRange(hsv, redGoalLower1, redGoalUpper1, redMask1);
        Core.inRange(hsv, redGoalLower2, redGoalUpper2, redMask2);
        Core.add(redMask1, redMask2, redMask);
        Imgproc.erode(redMask, redMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);
        Imgproc.dilate(redMask, redMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);

        // set the largest detection that was found to be the Red Goal detection
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(redMask, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < contours.size(); i++) {
            Detection newDetection = new Detection(input.size(),CV_MIN_GOAL_AREA,CV_MAX_GOAL_AREA);
            newDetection.setContour(contours.get(i));
            newDetection.draw(input, RED);
        }
        red.setMinArea(CV_MIN_GOAL_AREA);
        red.setMaxArea(CV_MAX_GOAL_AREA);
        red.setContour(getHighGoalContour(contours));

        // draw the Red Goal detection
        red.fill(input, RED);
    }

    // Update the Blue Goal Detection
    private void updateBlue(Mat input) {
        // take pixels that are in the color range and put them into a mask, eroding and dilating them to remove white noise
        Core.inRange(hsv, new Scalar(CAMERA_BLUE_GOAL_LOWER.get()), new Scalar(CAMERA_BLUE_GOAL_UPPER.get()), blueMask);
        Imgproc.erode(blueMask, blueMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);
        Imgproc.dilate(blueMask, blueMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);

        // set the largest detection that was found to be the Red Goal detection
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(blueMask, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < contours.size(); i++) {
            Detection newDetection = new Detection(input.size(),CV_MIN_GOAL_AREA, CV_MAX_GOAL_AREA);
            newDetection.setContour(contours.get(i));
            newDetection.draw(input, BLUE);
        }
        blue.setMinArea(CV_MIN_GOAL_AREA);
        blue.setMaxArea(CV_MAX_GOAL_AREA);
        blue.setContour(getHighGoalContour(contours));

        // draw the Blue Goal detection
        blue.fill(input, WHITE);
    }

    // Update the Red Powershot Detection
    private void updateRedPowershots(Mat input) {
        // check if the Red Goal is in sight (the powershots are based off of where the red goal is)
        if (!this.red.isValid()) {
            return;
        }

        // create a rectangle based off of the goal of where to look for the powershots
        Rect powershotRect = new Rect(
                (int) (red.getBottomLeftCornerPx().x - CV_POWERSHOT_DIMENSIONS.width + CV_POWERSHOT_OFFSET.x),
                (int) (red.getBottomLeftCornerPx().y + CV_POWERSHOT_OFFSET.y),
                (int) CV_POWERSHOT_DIMENSIONS.width,
                (int) CV_POWERSHOT_DIMENSIONS.height
        );

        // take pixels that are in the color range and put them into a mask
        redPowershotLower1 = new Scalar(CAMERA_RED_POWERSHOT_LOWER.getH(), CAMERA_RED_POWERSHOT_LOWER.getS(), CAMERA_RED_POWERSHOT_LOWER.getV());
        redPowershotUpper1 = new Scalar(180, CAMERA_RED_POWERSHOT_UPPER.getS(), CAMERA_RED_POWERSHOT_UPPER.getV());
        redPowershotLower2 = new Scalar(0, CAMERA_RED_POWERSHOT_LOWER.getS(), CAMERA_RED_POWERSHOT_LOWER.getV());
        redPowershotUpper2 = new Scalar(CAMERA_RED_POWERSHOT_UPPER.getH(), CAMERA_RED_POWERSHOT_UPPER.getS(), CAMERA_RED_POWERSHOT_UPPER.getV());

        Core.inRange(hsv, redPowershotLower1, redPowershotUpper1, redMask1);
        Core.inRange(hsv, redPowershotLower2, redPowershotUpper2, redMask2);
        Core.add(redMask1, redMask2, redMask);

        // make a new mask that will be used to find the contours of the Powershots
        Mat powershotMask = new Mat(hsv.size(), CvType.CV_8U);
        Imgproc.rectangle(powershotMask, powershotRect, WHITE, -1);
        Core.bitwise_and(redMask, powershotMask, powershotMask);

        // set the largest detection that was found to be the Powershots
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(powershotMask, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < contours.size(); i++) {
            if (OpenCVUtil.getCenterOfContour(contours.get(i)).x > red.getBottomLeftCornerPx().x ||
                    OpenCVUtil.getCenterOfContour(contours.get(i)).y < red.getBottomLeftCornerPx().y) {
                contours.remove(i);
                i--;
            }
        }
        redPowershots.setMinArea(CV_MIN_POWERSHOT_AREA);
        redPowershots.setMaxArea(CV_MAX_POWERSHOT_AREA);
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
                (int) (blue.getBottomRightCornerPx().x - CV_POWERSHOT_OFFSET.x),
                (int) (blue.getBottomRightCornerPx().y + CV_POWERSHOT_OFFSET.y),
                (int) CV_POWERSHOT_DIMENSIONS.width,
                (int) CV_POWERSHOT_DIMENSIONS.height
        );

        // take pixels that are in the color range and put them into a mask
        Core.inRange(hsv, new Scalar(CAMERA_BLUE_POWERSHOT_LOWER.get()), new Scalar(CAMERA_BLUE_POWERSHOT_UPPER.get()), blueMask);

        // make a new mask that will be used to find the contours of the Powershots
        Mat powershotMask = new Mat(hsv.size(), CvType.CV_8U);
        Imgproc.rectangle(powershotMask, powershotRect, WHITE, -1);
        Core.bitwise_and(blueMask, powershotMask, powershotMask);

        // set the largest detection that was found to be the Powershots
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(powershotMask, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < contours.size(); i++) {
            if (OpenCVUtil.getCenterOfContour(contours.get(i)).x < blue.getBottomRightCornerPx().x ||
                    OpenCVUtil.getCenterOfContour(contours.get(i)).y < blue.getBottomRightCornerPx().y) {
                contours.remove(i);
                i--;
            }
        }
        bluePowershots.setMinArea(CV_MIN_POWERSHOT_AREA);
        bluePowershots.setMaxArea(CV_MAX_POWERSHOT_AREA);
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
