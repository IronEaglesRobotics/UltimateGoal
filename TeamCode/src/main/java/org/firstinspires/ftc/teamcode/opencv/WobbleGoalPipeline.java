package org.firstinspires.ftc.teamcode.opencv;

import org.firstinspires.ftc.teamcode.CVHelpers;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.CVHelpers.getLargestContour;
import static org.firstinspires.ftc.teamcode.Constants.ANCHOR;
import static org.firstinspires.ftc.teamcode.Constants.BLUR_SIZE;
import static org.firstinspires.ftc.teamcode.Constants.ERODE_DILATE_ITERATIONS;
import static org.firstinspires.ftc.teamcode.Constants.MIN_STARTERSTACK_AREA;
import static org.firstinspires.ftc.teamcode.Constants.MIN_WOBBLEGOAL_AREA;
import static org.firstinspires.ftc.teamcode.Constants.ORANGE_LOWER;
import static org.firstinspires.ftc.teamcode.Constants.ORANGE_UPPER;
import static org.firstinspires.ftc.teamcode.Constants.RED;
import static org.firstinspires.ftc.teamcode.Constants.RED_LOWER_1;
import static org.firstinspires.ftc.teamcode.Constants.RED_LOWER_2;
import static org.firstinspires.ftc.teamcode.Constants.RED_UPPER_1;
import static org.firstinspires.ftc.teamcode.Constants.RED_UPPER_2;
import static org.firstinspires.ftc.teamcode.Constants.STARTERSTACK_LOCATION;
import static org.firstinspires.ftc.teamcode.Constants.STRUCTURING_ELEMENT;
import static org.firstinspires.ftc.teamcode.Constants.WHITE;

// Class for the pipeline that is used to detect the StarterStack
public class WobbleGoalPipeline extends OpenCvPipeline  {
    Mat blurred = new Mat();
    Mat hsv = new Mat();
    Mat redMask1 = new Mat();
    Mat redMask2 = new Mat();
    Mat redMask = new Mat();

    private Detection wobbleGoal;

    // Init
    @Override
    public void init(Mat input) {
        wobbleGoal = new Detection(input.size(), MIN_WOBBLEGOAL_AREA);
    }

    // Process each frame that is received from the webcam
    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.GaussianBlur(input, blurred, BLUR_SIZE, 0);
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);

        updateWobbleGoal(input);

        return input;
    }

    // Update the StarterStack Detection
    private void updateWobbleGoal(Mat input) {
        // take pixels that are in the color range and put them into a mask, eroding and dilating them to remove white noise
        Core.inRange(hsv, RED_LOWER_1, RED_UPPER_1, redMask1);
        Core.inRange(hsv, RED_LOWER_2, RED_UPPER_2, redMask2);
        Core.add(redMask1, redMask2, redMask);
        Imgproc.erode(redMask, redMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);
        Imgproc.dilate(redMask, redMask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);

        // set the largest detection that was found to be the Red Goal detection
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(redMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        wobbleGoal.setContour(getLargestContour(contours));

        // draw the Wobble Goal detection
        wobbleGoal.draw(input, RED);
    }

    // Get the WobbleGoal
    public Detection getWobbleGoal() {
        return wobbleGoal;
    }
}
